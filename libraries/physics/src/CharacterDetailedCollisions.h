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
#include "BulletUtil.h"


class CharacterDetailedCollisions {
public:

    struct CharacterDetailedRigidBody {
        const float DETAILED_COLLISION_RADIUS = 0.003f;
        const float DETAILED_MASS_KINEMATIC = 1.0f;
        const float LINEAR_VELOCITY_MULTIPLIER = 100.0f;

        CharacterDetailedRigidBody() { _rigidBody = nullptr; };
        CharacterDetailedRigidBody(std::vector<btVector3>& shapePoints);
        CharacterDetailedRigidBody(btVector3& bbox, btVector3& offset);

        void setTransform(float deltaTime, btTransform& transform);
        void cleanCollision();

        btVector3 _position;
        btVector3 _offset;
        btVector3 _previusPosition;
        btQuaternion _rotation;
        btRigidBody* _rigidBody{ nullptr };
        btDefaultMotionState* _motionState{ nullptr };

    };

    struct RayJointResult {
        int _intersectWithJoint{ -1 };
        float _distance{ 0.0f };
        glm::vec3 _intersectionPoint;
    };

    void setDynamicsWorld(btDynamicsWorld* world);
    void addRigidBody(std::vector<btVector3>& points);
    void addRigidBody(btVector3& bbox, btVector3& offset);
    void setRigidBodyTransform(float deltaTime, int jointIndex, glm::quat& rotation, glm::vec3& position);
    void setRigidBodyTransform(float deltaTime, int jointIndex, btTransform& transform);
    void updateCollisions();
    void removeCollisions();
    void cleanCollisions();
    bool hasRigidBody(int jointIndex);
    RayJointResult rayTest(const btVector3& origin, const btVector3& direction, const btScalar& length, const QVector<uint>& jointsToExclude = QVector<uint>()) const;
    const std::vector<CharacterDetailedRigidBody>& getRigidBodies() const { return _rigidBodies; };

private:
    std::vector<CharacterDetailedRigidBody> _rigidBodies;
    btDynamicsWorld* _world{ nullptr };
    bool _updated{ false };

};

#endif // hifi_CharacterDetailedCollisions_h