//
//  DetailedMotionState.cpp
//  interface/src/avatar/
//
//  Created by Luis Cuenca 12/18/2018
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "DetailedMotionState.h"

#include <PhysicsCollisionGroups.h>
#include <PhysicsEngine.h>
#include <PhysicsHelpers.h>


DetailedMotionState::DetailedMotionState(OtherAvatarPointer avatar, const btCollisionShape* shape) : ObjectMotionState(shape), _avatar(avatar) {
    assert(_avatar);
    _type = MOTIONSTATE_TYPE_AVATAR;
}

void DetailedMotionState::handleEasyChanges(uint32_t& flags) {
    ObjectMotionState::handleEasyChanges(flags);
    if (flags & Simulation::DIRTY_PHYSICS_ACTIVATION && !_body->isActive()) {
        _body->activate();
    }
}

bool DetailedMotionState::handleHardAndEasyChanges(uint32_t& flags, PhysicsEngine* engine) {
    return ObjectMotionState::handleHardAndEasyChanges(flags, engine);
}

DetailedMotionState::~DetailedMotionState() {
    assert(_avatar);
    _avatar = nullptr;
}

// virtual
uint32_t DetailedMotionState::getIncomingDirtyFlags() {
    return _body ? _dirtyFlags : 0;
}

void DetailedMotionState::clearIncomingDirtyFlags() {
    if (_body) {
        _dirtyFlags = 0;
    }
}

PhysicsMotionType DetailedMotionState::computePhysicsMotionType() const {
    // TODO?: support non-DYNAMIC motion for avatars? (e.g. when sitting)
    return MOTION_TYPE_DYNAMIC;
}

// virtual and protected
const btCollisionShape* DetailedMotionState::computeNewShape() {
    ShapeInfo shapeInfo;
    _avatar->computeShapeInfo(shapeInfo);
    return getShapeManager()->getShape(shapeInfo);
}

// virtual
bool DetailedMotionState::isMoving() const {
    return false;
}

// virtual
void DetailedMotionState::getWorldTransform(btTransform& worldTrans) const {
    worldTrans.setOrigin(glmToBullet(getObjectPosition()));
    worldTrans.setRotation(glmToBullet(getObjectRotation()));
    if (_body) {
        _body->setLinearVelocity(glmToBullet(getObjectLinearVelocity()));
        _body->setAngularVelocity(glmToBullet(getObjectAngularVelocity()));
    }
}

// virtual
void DetailedMotionState::setWorldTransform(const btTransform& worldTrans) {
    _body->setWorldTransform(worldTrans);
}

// These pure virtual methods must be implemented for each MotionState type
// and make it possible to implement more complicated methods in this base class.

// virtual
float DetailedMotionState::getObjectRestitution() const {
    return 0.5f;
}

// virtual
float DetailedMotionState::getObjectFriction() const {
    return 0.5f;
}

// virtual
float DetailedMotionState::getObjectLinearDamping() const {
    return 0.5f;
}

// virtual
float DetailedMotionState::getObjectAngularDamping() const {
    return 0.5f;
}

// virtual
glm::vec3 DetailedMotionState::getObjectPosition() const {
    return _avatar->getWorldPosition();
}

// virtual
glm::quat DetailedMotionState::getObjectRotation() const {
    return _avatar->getWorldOrientation();
}

// virtual
glm::vec3 DetailedMotionState::getObjectLinearVelocity() const {
    return _avatar->getWorldVelocity();
}

// virtual
glm::vec3 DetailedMotionState::getObjectAngularVelocity() const {
    // HACK: avatars use a capusle collision shape and their angularVelocity in the local simulation is unimportant.
    // Therefore, as optimization toward support for larger crowds we ignore it and return zero.
    //return _avatar->getWorldAngularVelocity();
    return glm::vec3(0.0f);
}

// virtual
glm::vec3 DetailedMotionState::getObjectGravity() const {
    return _avatar->getAcceleration();
}

// virtual
const QUuid DetailedMotionState::getObjectID() const {
    return _avatar->getSessionUUID();
}

QString DetailedMotionState::getName() const {
    return _avatar->getName();
}

// virtual
QUuid DetailedMotionState::getSimulatorID() const {
    return _avatar->getSessionUUID();
}

// virtual
void DetailedMotionState::computeCollisionGroupAndMask(int32_t& group, int32_t& mask) const {
    group = BULLET_COLLISION_GROUP_OTHER_AVATAR;
    mask = Physics::getDefaultCollisionMask(group);
}

// virtual
float DetailedMotionState::getMass() const {
    return _avatar->computeMass();
}

void DetailedMotionState::setRigidBody(btRigidBody* body) {
    ObjectMotionState::setRigidBody(body);
    if (_body) {
        // remove angular dynamics from this body
        _body->setAngularFactor(0.0f);
    }
}

void DetailedMotionState::setShape(const btCollisionShape* shape) {
    ObjectMotionState::setShape(shape);
}
