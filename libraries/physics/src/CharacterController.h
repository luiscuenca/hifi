//
//  CharacterControllerInterface.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2015.10.21
//  Copyright 2015 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_CharacterController_h
#define hifi_CharacterController_h

#include <assert.h>
#include <stdint.h>
#include <atomic>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Character/btCharacterControllerInterface.h>

#include <GLMHelpers.h>
#include <NumericalConstants.h>
#include <PhysicsCollisionGroups.h>

#include "BulletUtil.h"
#include "CharacterGhostObject.h"
#include "AvatarConstants.h" 
#include "EntityMotionState.h"

const uint32_t PENDING_FLAG_ADD_TO_SIMULATION = 1U << 0;
const uint32_t PENDING_FLAG_REMOVE_FROM_SIMULATION = 1U << 1;
const uint32_t PENDING_FLAG_UPDATE_SHAPE = 1U << 2;
const uint32_t PENDING_FLAG_JUMP = 1U << 3;
const uint32_t PENDING_FLAG_UPDATE_COLLISION_GROUP = 1U << 4;
const uint32_t PENDING_FLAG_RECOMPUTE_FLYING = 1U << 5;
const float DEFAULT_MIN_FLOOR_NORMAL_DOT_UP = cosf(PI / 3.0f);

class btRigidBody;
class btCollisionWorld;
class btDynamicsWorld;

//#define DEBUG_STATE_CHANGE

const btScalar MAX_CHARACTER_MOTOR_TIMESCALE = 60.0f; // one minute
const btScalar MIN_CHARACTER_MOTOR_TIMESCALE = 0.05f;

class CharacterController : public btCharacterControllerInterface {

public:
    struct CharacterDetailedRigidBody {
        const float DETAILED_COLLISION_RADIUS = 0.003f;
        const float DETAILED_MASS_KINEMATIC = 1.0f;
        const float LINEAR_VELOCITY_MULTIPLIER = 100.0f;

        CharacterDetailedRigidBody() { _rigidBody = nullptr; };
        CharacterDetailedRigidBody(std::vector<btVector3>& shapePoints);

        void setTransform(float deltaTime, btTransform& transform);
        void cleanCollision();

        btVector3  _position;
        btVector3 _previusPosition;
        btQuaternion _rotation;
        btRigidBody* _rigidBody { nullptr };
        btDefaultMotionState* _motionState { nullptr };
    };

    struct CharacterDetailedCollisions {
        void setDynamicsWorld(btDynamicsWorld* world);
        void addRigidBody(std::vector<btVector3>& points);
        void setRigidBodyTransform(float deltaTime, int jointIndex, glm::quat& rotation, glm::vec3& position);
        void setRigidBodyTransform(float deltaTime, int jointIndex, btTransform& transform);
        void updateCollisions();
        void removeCollisions();
        void cleanCollisions();
        bool hasRigidBody(int jointIndex);

        std::vector<CharacterDetailedRigidBody> _rigidBodies;
        btDynamicsWorld* _world { nullptr };
        bool _updated { false };
    };

    CharacterController();
    virtual ~CharacterController();
    bool needsRemoval() const;
    bool needsAddition() const;
    virtual void setDynamicsWorld(btDynamicsWorld* world);
    btCollisionObject* getCollisionObject() { return _rigidBody; }

    void setGravity(float gravity);
    float getGravity();
    void recomputeFlying();

    virtual void updateShapeIfNecessary() = 0;

    // overrides from btCharacterControllerInterface
    virtual void setWalkDirection(const btVector3 &walkDirection) override { assert(false); }
    virtual void setVelocityForTimeInterval(const btVector3 &velocity, btScalar timeInterval) override { assert(false); }
    virtual void reset(btCollisionWorld* collisionWorld) override { }
    virtual void warp(const btVector3& origin) override { }
    virtual void debugDraw(btIDebugDraw* debugDrawer) override { }
    virtual void setUpInterpolate(bool value) override { }
    virtual void updateAction(btCollisionWorld* collisionWorld, btScalar deltaTime) override;
    virtual void preStep(btCollisionWorld *collisionWorld) override;
    virtual void playerStep(btCollisionWorld *collisionWorld, btScalar dt) override;
    virtual bool canJump() const override { assert(false); return false; } // never call this
    virtual void jump() override;
    virtual bool onGround() const override;

    void clearMotors();
    void addMotor(const glm::vec3& velocity, const glm::quat& rotation, float horizTimescale, float vertTimescale = -1.0f);
    void applyMotor(int index, btScalar dt, btVector3& worldVelocity, std::vector<btVector3>& velocities, std::vector<btScalar>& weights);
    void setStepUpEnabled(bool enabled) { _stepUpEnabled = enabled; }
    void computeNewVelocity(btScalar dt, btVector3& velocity);
    void computeNewVelocity(btScalar dt, glm::vec3& velocity);
    void setScaleFactor(btScalar scaleFactor) { _scaleFactor = scaleFactor; }

    // HACK for legacy 'thrust' feature
    void setLinearAcceleration(const glm::vec3& acceleration) { _linearAcceleration = glmToBullet(acceleration); }

    void preSimulation();
    void postSimulation();

    void setPositionAndOrientation( const glm::vec3& position, const glm::quat& orientation);
    void getPositionAndOrientation(glm::vec3& position, glm::quat& rotation) const;

    void setParentVelocity(const glm::vec3& parentVelocity);
    void setFollowParameters(const glm::mat4& desiredWorldMatrix, float timeRemaining);
    float getFollowTime() const { return _followTime; }
    glm::vec3 getFollowLinearDisplacement() const;
    glm::quat getFollowAngularDisplacement() const;
    glm::vec3 getFollowVelocity() const;

    glm::vec3 getLinearVelocity() const;
    glm::vec3 getVelocityChange() const;

    float getCapsuleRadius() const { return _radius; }
    float getCapsuleHalfHeight() const { return _halfHeight; }
    glm::vec3 getCapsuleLocalOffset() const { return _shapeLocalOffset; }

    enum class State {
        Ground = 0,
        Takeoff,
        InAir,
        Hover
    };

    State getState() const { return _state; }
    void updateState();

    void setLocalBoundingBox(const glm::vec3& minCorner, const glm::vec3& scale);

    bool isEnabledAndReady() const { return _dynamicsWorld; }
    bool isStuck() const { return _isStuck; }

    void setCollisionless(bool collisionless);
    int16_t computeCollisionGroup() const;
    void handleChangedCollisionGroup();

    bool getRigidBodyLocation(glm::vec3& avatarRigidBodyPosition, glm::quat& avatarRigidBodyRotation);

    void setFlyingAllowed(bool value);
    void setCollisionlessAllowed(bool value);


protected:
#ifdef DEBUG_STATE_CHANGE
    void setState(State state, const char* reason);
#else
    void setState(State state);
#endif

    virtual void updateMassProperties() = 0;
    void updateCurrentGravity();
    void updateUpAxis(const glm::quat& rotation);
    bool checkForSupport(btCollisionWorld* collisionWorld);
    void updatePhysicsState();

protected:
    struct CharacterMotor {
        CharacterMotor(const glm::vec3& vel, const glm::quat& rot, float horizTimescale, float vertTimescale = -1.0f);

        btVector3 velocity { btVector3(0.0f, 0.0f, 0.0f) }; // local-frame
        btQuaternion rotation; // local-to-world
        btScalar hTimescale { MAX_CHARACTER_MOTOR_TIMESCALE }; // horizontal
        btScalar vTimescale { MAX_CHARACTER_MOTOR_TIMESCALE }; // vertical
    };

    std::vector<CharacterMotor> _motors;
    CharacterGhostObject _ghost;
    btVector3 _currentUp;
    btVector3 _targetVelocity;
    btVector3 _parentVelocity;
    btVector3 _preSimulationVelocity;
    btVector3 _velocityChange;
    btTransform _followDesiredBodyTransform;
    btScalar _followTimeRemaining;
    btTransform _characterBodyTransform;
    btVector3 _position;
    btQuaternion _rotation;

    glm::vec3 _shapeLocalOffset;

    glm::vec3 _boxScale; // used to compute capsule shape

    quint64 _rayHitStartTime;
    quint64 _takeoffToInAirStartTime;
    quint64 _jumpButtonDownStartTime;
    quint32 _jumpButtonDownCount;
    quint32 _takeoffJumpButtonID;

    // data for walking up steps
    btVector3 _stepPoint { 0.0f, 0.0f, 0.0f };
    btVector3 _stepNormal { 0.0f, 0.0f, 0.0f };
    bool _steppingUp { false };
    btScalar _stepHeight { 0.0f };
    btScalar _minStepHeight { 0.0f };
    btScalar _maxStepHeight { 0.0f };
    btScalar _minFloorNormalDotUp { DEFAULT_MIN_FLOOR_NORMAL_DOT_UP };

    btScalar _halfHeight { 0.0f };
    btScalar _radius { 0.0f };

    btScalar _floorDistance;
    bool _stepUpEnabled { true };
    bool _hasSupport;

    btScalar _currentGravity { 0.0f };
    btScalar _gravity { DEFAULT_AVATAR_GRAVITY };

    btScalar _followTime;
    btVector3 _followLinearDisplacement;
    btQuaternion _followAngularDisplacement;
    btVector3 _linearAcceleration;

    State _state;
    bool _isPushingUp;
    bool _isStuck { false };

    btDynamicsWorld* _dynamicsWorld { nullptr };
    btRigidBody* _rigidBody { nullptr };
    uint32_t _pendingFlags { 0 };
    uint32_t _previousFlags { 0 };
    CharacterDetailedCollisions _detailedCollisions;
    std::map<QUuid, CharacterDetailedCollisions> _otherCharactersDetailedCollisions;

    bool _flyingAllowed { true };
    bool _collisionlessAllowed { true };
    bool _collisionless { false };

    btScalar _scaleFactor { 1.0f };
};

#endif // hifi_CharacterController_h
