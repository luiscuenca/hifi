//
//  AvatarJointMotionState.cpp
//  interface/src/avatar/
//
//  Created by Andrew Meadows 2015.05.14
//  Copyright 2015 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "AvatarJointMotionState.h"

#include <PhysicsCollisionGroups.h>
#include <PhysicsEngine.h>
#include <PhysicsHelpers.h>


AvatarJointMotionState::AvatarJointMotionState(AvatarSharedPointer avatar, int jointIndex, const btCollisionShape* shape) : 
    ObjectMotionState(shape), _jointIndex(jointIndex), _avatar(avatar) {
    assert(_avatar);
    assert(_jointIndex >= 0 && _jointIndex < _avatar->getJointCount());
    _type = MOTIONSTATE_TYPE_AVATAR;
}

AvatarJointMotionState::~AvatarJointMotionState() {
    assert(_avatar);
    _avatar = nullptr;
}

// virtual
uint32_t AvatarJointMotionState::getIncomingDirtyFlags() {
    return _body ? _dirtyFlags : 0;
}

void AvatarJointMotionState::clearIncomingDirtyFlags() {
    if (_body) {
        _dirtyFlags = 0;
    }
}

PhysicsMotionType AvatarJointMotionState::computePhysicsMotionType() const {
    // TODO?: support non-DYNAMIC motion for avatars? (e.g. when sitting)
    return MOTION_TYPE_DYNAMIC;
}

// virtual and protected
const btCollisionShape* AvatarJointMotionState::computeNewShape() {
    ShapeInfo shapeInfo;
    std::static_pointer_cast<Avatar>(_avatar)->computeShapeInfo(_jointIndex, shapeInfo);
    return getShapeManager()->getShape(shapeInfo);
}

// virtual
bool AvatarJointMotionState::isMoving() const {
    return false;
}

// virtual
void AvatarJointMotionState::getWorldTransform(btTransform& worldTrans) const {
    worldTrans.setOrigin(glmToBullet(getObjectPosition()));
    worldTrans.setRotation(glmToBullet(getObjectRotation()));
    if (_body) {
        _body->setLinearVelocity(glmToBullet(getObjectLinearVelocity()));
        _body->setAngularVelocity(glmToBullet(getObjectLinearVelocity()));
    }
}

// virtual
void AvatarJointMotionState::setWorldTransform(const btTransform& worldTrans) {
    // HACK: The PhysicsEngine does not actually move OTHER avatars -- instead it slaves their local RigidBody to the transform
    // as specified by a remote simulation.  However, to give the remote simulation time to respond to our own objects we tie
    // the other avatar's body to its true position with a simple spring. This is a HACK that will have to be improved later.
    const float SPRING_TIMESCALE = 0.5f;
    float tau = PHYSICS_ENGINE_FIXED_SUBSTEP / SPRING_TIMESCALE;
    btVector3 currentPosition = worldTrans.getOrigin();
    btVector3 targetPosition = glmToBullet(getObjectPosition());
    btTransform newTransform;
    newTransform.setOrigin((1.0f - tau) * currentPosition + tau * targetPosition);
    newTransform.setRotation(glmToBullet(getObjectRotation()));
    _body->setWorldTransform(newTransform);
    _body->setLinearVelocity(glmToBullet(getObjectLinearVelocity()));
    _body->setAngularVelocity(glmToBullet(getObjectLinearVelocity()));
}

// These pure virtual methods must be implemented for each MotionState type
// and make it possible to implement more complicated methods in this base class.

// virtual
float AvatarJointMotionState::getObjectRestitution() const {
    return 0.5f;
}

// virtual
float AvatarJointMotionState::getObjectFriction() const {
    return 0.5f;
}

// virtual
float AvatarJointMotionState::getObjectLinearDamping() const {
    return 0.5f;
}

// virtual
float AvatarJointMotionState::getObjectAngularDamping() const {
    return 0.5f;
}

// virtual
glm::vec3 AvatarJointMotionState::getObjectPosition() const {
    return std::static_pointer_cast<Avatar>(_avatar)->getJointPosition(_jointIndex);
}

// virtual
glm::quat AvatarJointMotionState::getObjectRotation() const {
    return std::static_pointer_cast<Avatar>(_avatar)->getWorldOrientation() * std::static_pointer_cast<Avatar>(_avatar)->getAbsoluteJointRotationInObjectFrame(_jointIndex);
}

// virtual
glm::vec3 AvatarJointMotionState::getObjectLinearVelocity() const {
    return (100.0f * (getObjectPosition() - bulletToGLM(getRigidBody()->getWorldTransform().getOrigin())));
}

// virtual
glm::vec3 AvatarJointMotionState::getObjectAngularVelocity() const {
    return _avatar->getWorldAngularVelocity();
}

// virtual
glm::vec3 AvatarJointMotionState::getObjectGravity() const {
    return glm::vec3();
}

// virtual
const QUuid AvatarJointMotionState::getObjectID() const {
    return _avatar->getSessionUUID();
}

// virtual
QUuid AvatarJointMotionState::getSimulatorID() const {
    return _avatar->getSessionUUID();
}

// virtual
void AvatarJointMotionState::computeCollisionGroupAndMask(int16_t& group, int16_t& mask) const {
    group = BULLET_COLLISION_GROUP_OTHER_AVATAR;
    mask = Physics::getDefaultCollisionMask(group);
}

