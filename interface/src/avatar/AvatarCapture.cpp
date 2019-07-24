#include "AvatarCapture.h"
#include "AudioClient.h"
#include "RecordingScriptingInterface.h"

const int MAX_RECORDING_TIME = 10000; //ms

QString AvatarCapture::getDefaultCaptureSaveDirectory() {
    QString directory = PathUtils::getAppLocalDataPath() + "Avatar Captures/";
    if (!QDir(directory).exists()) {
        QDir().mkdir(directory);
    }
    return directory;
}

bool AvatarCapture::captureAvatar(const QUuid& avatarID, const QString& displayName) {
    if (!_isRecordingAvatar && _avatarToRecordID.isNull()) {
        _avatarToRecordID = avatarID;
        _avatarInfo.sessionId = avatarID;
        _avatarInfo.displayName = displayName;
        return prepareFilesForCapture();
    }
    return false;
}

int AvatarCapture::getRecordingTimeLeft() {
    if (_isRecordingAvatar) {
        return MAX_RECORDING_TIME - _recordingTime;
    } else {
        return 0;
    }
}

bool AvatarCapture::prepareFilesForCapture(){
    auto audio = DependencyManager::get<AudioClient>().data();
    static const auto unixEpoch = std::chrono::system_clock::from_time_t(0);
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - unixEpoch).count();
    QString directory = getDefaultCaptureSaveDirectory();
    _captureName = QString::number(time);
    _captureSoundFileName = directory + _captureName + ".wav";
    _capturePacketsFileName = directory + _captureName + ".pkt";
    _capturePacketsFile.setFileName(_capturePacketsFileName);
    if (!audio->startRecording(_captureSoundFileName) || !_capturePacketsFile.open(QIODevice::WriteOnly)) {
        return false;
    }
    return true;
}

AvatarCapture::CaptureState AvatarCapture::updateCaptureState(const OtherAvatarPointer& avatar, uint64_t timeNow, float deltaTime) {
    _state = CaptureState::Idle;
    if (!_isPlayingBack) {
        if (_avatarToRecordID == avatar->getSessionUUID()) {
            _state = CaptureState::Recording;
            if (!_isRecordingAvatar) {
                _avatarToRecordURL = avatar->getSkeletonModelURL();
                _isRecordingAvatar = true;
                _startRecordingAvatarTimestamp = timeNow;
            } else {
                QByteArray lastPacket;
                bool writeSuccess = true;
                if (avatar->getLastAvatarPacket(lastPacket)) {
                    int deltaTime = _capturePacketsFile.pos() == 0 ? 0 : int(timeNow - _lastRecordingTimestamp);
                    writeSuccess = writeNextCapturedPacket(lastPacket, deltaTime);
                }
                _recordingTime = (int)(timeNow - _startRecordingAvatarTimestamp) * 0.001;
                _lastRecordingTimestamp = timeNow;
                if (!writeSuccess || _recordingTime > MAX_RECORDING_TIME) {
                    QVariantMap res;
                    res.insert("avatarID", _avatarToRecordID);
                    res.insert("packetsFileName", _capturePacketsFileName);
                    res.insert("soundFileName", _captureSoundFileName);

                    _avatarToRecordID = QUuid();
                    _isRecordingAvatar = false;
                    _recordingTime = 0;
                    DependencyManager::get<AudioClient>().data()->stopRecording();
                    _capturePacketsFile.flush();
                    _capturePacketsFile.close();
                    auto nodeList = DependencyManager::get<NodeList>();
                    connect(nodeList.data(), &NodeList::usernameFromIDReply, this, &AvatarCapture::saveMetadataWithUsername);
                    nodeList->requestUsernameFromSessionID(_avatarInfo.sessionId);
                    emit recordFinish(res);
                    _state = CaptureState::DoneRecording;
                }
            }
        }
    } else if (_avatarToPlaybackID == avatar->getSessionUUID()) {
        _state = CaptureState::Playing;      
        if (_playbackDeltaTime >= _lastDeltaTime) {
            if (readNextCapturedPacket()) {
                avatar->parseDataFromBuffer(_lastReadPacket);
            } else {
                _isPlayingBack = false;
                _lastDeltaTime = 0;
                _lastReadPacket.clear();
                QString captureName = QFileInfo(_capturePacketsFile.fileName()).fileName().section(".", 0, 0);
                writeMetadata(getDefaultCaptureSaveDirectory() + captureName + ".txt", _avatarToPlaybackID.toString(), _avatarInfo.userName, false);
                _state = CaptureState::DonePlaying;
            }
            _playbackDeltaTime = 0;
        } else {
            _playbackDeltaTime += deltaTime * 1000000;
        }
    }
    return _state;
}

void AvatarCapture::saveMetadataWithUsername(const QString& nodeID, const QString& username,
                                             const QString& machineFingerprint, bool isAdmin) {
        QString metaPath = getDefaultCaptureSaveDirectory() + _captureName + ".txt";
        if (!writeMetadata(metaPath, _avatarInfo.sessionId, username != "" ? username : _avatarInfo.displayName, true)) {
            qCDebug(interfaceapp) << "Error saving capture metadata: " << metaPath;
        }
        auto nodeList = DependencyManager::get<NodeList>();
        disconnect(nodeList.data(), &NodeList::usernameFromIDReply, this, &AvatarCapture::saveMetadataWithUsername);
}

void AvatarCapture::includeAvatarInHash(AvatarHash& avatarHash) {
    if (_isPlayingBack) {
        if (!std::static_pointer_cast<Avatar>(_recordedAvatar)->isInitialized()) {
            std::static_pointer_cast<Avatar>(_recordedAvatar)->init();
        }
        avatarHash.insert(_avatarToPlaybackID, _recordedAvatar);
    }
}

bool AvatarCapture::canPlay(const QString& filename) {
    bool canPlay = !_isPlayingBack && !_avatarToRecordURL.isEmpty() && _recordedAvatar == nullptr;
    if (canPlay) {
        if (_capturePacketsFile.isOpen()) {
            _capturePacketsFile.close();
        }
        _capturePacketsFile.setFileName(filename);
        return _capturePacketsFile.open(QIODevice::ReadOnly);
    }    
    return false;
}

void AvatarCapture::setActor(AvatarSharedPointer& actor) {
    _recordedAvatar = actor;
    if (actor != nullptr) {
        auto avatar = std::static_pointer_cast<Avatar>(_recordedAvatar);
        avatar->setSkeletonModelURL(_avatarToRecordURL);
        _avatarToPlaybackID = avatar->getSessionUUID();
    }
}

void AvatarCapture::play(const glm::vec3& pos) {
    if (_recordedAvatar != nullptr) {
        _isPlayingBack = true;
    }
}

bool AvatarCapture::writeNextCapturedPacket(const QByteArray& packet, int deltaTime) {
    if (_capturePacketsFile.isOpen()) {
        QDataStream out(&_capturePacketsFile);
        out.setVersion(QDataStream::Qt_5_9);
        out.setByteOrder(QDataStream::LittleEndian);
        out << packet.size();
        out << deltaTime;
        out << packet;
    } else {
        return false;
    }
    return true;
}

bool AvatarCapture::readNextCapturedPacket() {
    if (_capturePacketsFile.isOpen() && !_capturePacketsFile.atEnd()) {
        QDataStream out(&_capturePacketsFile);
        out.setVersion(QDataStream::Qt_5_9);
        out.setByteOrder(QDataStream::LittleEndian);
        int size;
        out >> size;
        out >> _lastDeltaTime;
        _lastReadPacket = QByteArray();
        _lastReadPacket.resize(size);
        out >> _lastReadPacket;
        return true;
    } 
    return false;
}

bool AvatarCapture::writeMetadata(const QString& path, const QUuid& owner, const QString& userName, bool isNew) {
    QFile metaFile(path);
    if (metaFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream stream(&metaFile);
        stream << owner.toString() << "\n";
        stream << userName << "\n";
        stream << (isNew ? "true" : "false");
        metaFile.flush();
        metaFile.close();
        return true;
    }
    return false;
}

bool AvatarCapture::readMetaFile(const QString& path, QUuid& owner, QString& userName, bool& isNew) {
    QFile metaFile(path);
    bool success = false;
    if (metaFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream stream(&metaFile);
        QString ownerId;
        stream >> ownerId;
        owner = QUuid(ownerId);
        if (!owner.isNull()) {
            stream >> userName;
            QString isNewString;
            stream >> isNewString;
            isNew = isNewString == "true";
            bool isFalse = isNewString == "false";
            success = isNew || isFalse;
        }
        metaFile.flush();
        metaFile.close();
    }
    return success;
}

QVariantMap AvatarCapture::getCaptures() {
    QDir dir(getDefaultCaptureSaveDirectory());
    auto captureFiles = dir.entryList(QStringList() << "*.pkt", QDir::Filter::Files, QDir::SortFlag::Name);
    QVariantMap res;
    for (int i = 0; i < captureFiles.size(); i++) {
        QVariantMap capture;
        auto fileParts = captureFiles[i].split(".");
        QString captureName = captureFiles[i].section(".", 0, 0);
        QFileInfo metaFileInfo(dir.absoluteFilePath(captureName + ".txt"));
        QUuid owner;
        bool isNew;
        QString userName;
        qDebug() << "Reading: " << metaFileInfo.absoluteFilePath();
        if (!metaFileInfo.exists() || !readMetaFile(metaFileInfo.absoluteFilePath(), owner, userName, isNew)) {
            continue;
        }
        QFileInfo packetFileInfo(dir.absoluteFilePath(captureName + ".pkt"));
        QFileInfo audioFileInfo(dir.absoluteFilePath(captureName + ".wav"));

        capture.insert("packets", packetFileInfo.absoluteFilePath());
        capture.insert("audio", audioFileInfo.exists() ? audioFileInfo.absoluteFilePath() : "");
        capture.insert("hasAudio", audioFileInfo.exists());
        capture.insert("isNew", isNew);
        QString ownerId = owner.toString();
        capture.insert("owner", ownerId);
        capture.insert("userName", userName);
        if (!res.contains(userName)) {
            res.insert(userName, QList<QVariant>());
        }
        auto ownerArray = res[userName].toList();
        ownerArray.push_back(capture);
        res[userName] = ownerArray;
    }
    return res;
}