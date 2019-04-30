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
    enum CaptureState {
        Idle = 0,
        Recording,
        Playing,
        DoneRecording,
        DonePlaying
    };
    QString getDefaultCaptureSaveDirectory();
    bool captureAvatarByID(const QUuid& avatarID);
    CaptureState updateCaptureState(const OtherAvatarPointer& avatar, uint64_t timeNow, float deltaTime);
    void includeAvatarInHash(AvatarHash& avatarHash);
    bool canPlay(const QString& filename);
    void setActor(AvatarSharedPointer& actor);
    void play(const glm::vec3& pos);
    int getRecordingTimeLeft();
    bool writeNextCapturedPacket(const QByteArray& packet, int deltaTime);
    bool readNextCapturedPacket();

signals:
    void recordFinish(const QVariantMap& res);

private:
    bool prepareFilesForCapture();
    bool setMetadata(const QString& name, const QUuid& owner, bool isNew);
    std::shared_ptr<AvatarData> _recordedAvatar { nullptr };
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
};

#endif  // hifi_AvatarCapture_h