//
//  CharacterMultiBody.h
//  libraries/physics/src
//
//  Created by Luis Cuenca 8/4/2018
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_CharacterMultiBody_h
#define hifi_CharacterMultiBody_h

#include <stdint.h>

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <GLMHelpers.h>
#include <ScriptValueUtils.h>
#include "BulletUtil.h"
#include "PhysicsCollisionGroups.h"

enum BodyType {
    Box = 0,
    Sphere
};

enum ConstraintType {
    Spherical = 0,
    Planar,
    Prismatic,
    Revolute
};

struct CharacterRigidBody {
    CharacterRigidBody() :
        _bodyType(BodyType::Box) { };
    int _parentIndex;
    BodyType _bodyType;
    ConstraintType _constraintType;
    int _linkIndex;
    std::vector<btVector3> _spheresPositions;
    std::vector<btScalar> _spheresRadius;
    btVector3 _boxHalfBounds;
    btVector3 _defaultPosition;
    btQuaternion _defaultRotation;
    btVector3 _hingeJointAxis;
    btMultiBodyLinkCollider* _body { nullptr };
};

class CharacterMultiBody {
public:
    void createAvatarMultiBody();
    void setAvatarMultiBodyPosition(float deltaTime, const glm::vec3& newPosition);
    void cleanAvatarMultiBody();
    void removeAvatarMultiBody();
    void updateAvatarMultiBody();
    void setupMultiBody();
    void setDynamicsWorld(btMultiBodyDynamicsWorld* world);

private:
    void addAvatarMultiBodyColliders();
    void setFlags(int32_t group, int32_t mask) { _group = group; _mask = mask; }
    CharacterRigidBody getJointConfiguration(const glm::vec3& translation, const glm::quat& rotation, float startRadius, float endRadius, int parentIndex, bool collidable = true);
    btMultiBodyDynamicsWorld* _world { nullptr };
    btMultiBody* _avatarMultiBody { nullptr };
    bool _multiUpdated { false };
    int32_t _group { BULLET_COLLISION_GROUP_DYNAMIC };
    int32_t _mask { BULLET_COLLISION_MASK_DYNAMIC };
    bool _damping { true };
    bool _gyro { true };
    int _numLinks { 0 };
    bool _spherical { true };				//set it ot false -to use 1DoF hinges instead of 3DoF sphericals		
    bool _multibodyOnly { false };
    bool _canSleep { false };
    bool _collidable { true };
    bool _selfCollide { true };
    bool _multibodyConstraint { false };
    std::vector<CharacterRigidBody> _rigidBodies;
    bool _setted { false };
};

#endif // hifi_CharacterMultiBody_h
