//
//  AvatarCapture.h
//
//
//  Created by Luis Cuenca on 04/30/2019.
//  Copyright 2019 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#pragma once
#ifndef hifi_AvatarCapture_h
#define hifi_AvatarCapture_h

#include <QtCore/QObject>
#include <QtCore/QSharedPointer>
#include "OtherAvatar.h"

class AvatarCapture : public QObject {
    Q_OBJECT
public:
    struct AvatarMiniInfo {
        QString displayName;
        QString userName;
        QUuid sessionId;
    };
    enum CaptureState {
        Idle = 0,
        Recording,
        Playing,
        DoneRecording,
        DonePlaying
    };
    QString getDefaultCaptureSaveDirectory();
    bool captureAvatar(const QUuid& avatarID, const QString& displayName);
    CaptureState updateCaptureState(const OtherAvatarPointer& avatar, uint64_t timeNow, float deltaTime);
    void includeAvatarInHash(AvatarHash& avatarHash);
    bool canPlay(const QString& filename);
    void setActor(AvatarSharedPointer& actor);
    void play(const glm::vec3& pos);
    int getRecordingTimeLeft();
    bool writeNextCapturedPacket(const QByteArray& packet, int deltaTime);
    bool readNextCapturedPacket();
    bool readMetaFile(const QString& filePath, QUuid& owner, QString& userName, bool& isNew);
    QVariantMap getCaptures();

signals:
    void recordFinish(const QVariantMap& res);

public slots:
    void saveMetadataWithUsername(const QString& nodeID, const QString& username,
                                  const QString& machineFingerprint, bool isAdmin);

private:
    bool prepareFilesForCapture();
    bool writeMetadata(const QString& name, const QUuid& owner, const QString& userName, bool isNew);
    std::shared_ptr<AvatarData> _recordedAvatar { nullptr };
    AvatarMiniInfo _avatarInfo;
    QUuid _avatarToRecordID;
    QUuid _avatarToPlaybackID;
    QUrl _avatarToRecordURL;
    QString _captureSoundFileName;
    QString _capturePacketsFileName;
    int _recordingTime{ 0 };
    bool _isRecordingAvatar{ false };
    uint64_t _lastRecordingTimestamp{ 0 };
    uint64_t _startRecordingAvatarTimestamp{ 0 };
    int _playbackDeltaTime{ 0 };
    int _lastDeltaTime{ 0 };
    bool _isPlayingBack{ false };
    QByteArray _lastReadPacket;
    CaptureState _state{ CaptureState::Idle };
    glm::vec3 _playPosition;
    QFile _capturePacketsFile;
    QString _captureName;
};

#endif  // hifi_AvatarCapture_h