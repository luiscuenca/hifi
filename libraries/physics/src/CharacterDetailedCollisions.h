//
//  CharacterDetailedCollisions.h
//  libraries/physics/src
//
//  Created by Luis Cuenca 5/11/2018
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_CharacterDetailedCollisions_h
#define hifi_CharacterDetailedCollisions_h

#include <stdint.h>

#include <btBulletDynamicsCommon.h>
#include <GLMHelpers.h>
#include <ScriptValueUtils.h>
#include "BulletUtil.h"
#include "PhysicsCollisionGroups.h"


class CharacterDetailedCollisions {
public:

    struct CharacterDetailedRigidBody {
        enum delta {
            POSITION = 0,
            FRAME
        };
        enum type {
            KINEMATIC = 0,
            DYNAMIC
        };
        const float DETAILED_COLLISION_RADIUS = 0.003f;
        const float DETAILED_MASS_KINEMATIC = 2.0f;
        const float DETAILED_MASS_DYNAMIC = 1.0f;
        const float LINEAR_VELOCITY_MULTIPLIER = 100.0f;
        
        CharacterDetailedRigidBody() { _rigidBody = nullptr; };
        CharacterDetailedRigidBody(std::vector<btVector3>& shapePoints, int type);
        CharacterDetailedRigidBody(btVector3& bbox, btVector3& offset);

        void setTransform(float deltaTime, btTransform& transform);
        void cleanCollision();

        btTransform _lastTransform;
        btVector3 _offset;
        btRigidBody* _rigidBody { nullptr };
        btDefaultMotionState* _motionState { nullptr };

        int _init { 0 };
        bool _colliding { false };

        int _forceDeltaType { delta::FRAME };
        int _velocityDeltaType { delta::FRAME };
        int _impulseDeltaType { delta::POSITION };

        float _forceDeltaFrameMult { 0.0f };
        float _velocityDeltaFrameMult { 0.5f };
        float _impulseDeltaFrameMult { 0.0f };
        
        float _forceDeltaPositionMult { 0.0f };
        float _velocityDeltaPositionMult { 0.0f };
        float _impulseDeltaPositionMult { 0.2f };

        bool _applyLinearVelocity { true };
        bool _applyImpulse { true };
        bool _applyForce { false };

        bool _attenuate { true };
        int _type { type::KINEMATIC };
    };

    struct RayJointResult {
        int _intersectWithJoint{ -1 };
        float _distance{ 0.0f };
        glm::vec3 _intersectionPoint;
    };
    void setCollisionGroup(int16_t group) { _group = group; };
    void setCollisionMask(int16_t mask) { _mask = mask; };
    void setDynamicsWorld(btDynamicsWorld* world);
    void addRigidBody(std::vector<btVector3>& points, int type);
    void addRigidBody(btVector3& bbox, btVector3& offset);
    void setRigidBodyTransform(float deltaTime, int jointIndex, glm::quat& rotation, glm::vec3& position);
    void setRigidBodyTransform(float deltaTime, int jointIndex, btTransform& transform);
    void updateCollisions();
    void removeCollisions();
    void cleanCollisions();
    bool hasRigidBody(int jointIndex);
    void configurePhysics(const std::vector<float>& args);
    void getPhysicsArgs(std::vector<float>& args);
    RayJointResult rayTest(const btVector3& origin, const btVector3& direction, const btScalar& length, const QVector<uint>& jointsToExclude = QVector<uint>()) const;
    const std::vector<CharacterDetailedRigidBody>& getRigidBodies() const { return _rigidBodies; };
    
private:
    std::vector<CharacterDetailedRigidBody> _rigidBodies;
    btDynamicsWorld* _world { nullptr };
    bool _updated { false };
    int16_t _group { BULLET_COLLISION_GROUP_COLLISIONLESS };
    int16_t _mask { BULLET_COLLISION_MASK_COLLISIONLESS };

};

#endif // hifi_CharacterDetailedCollisions_h
