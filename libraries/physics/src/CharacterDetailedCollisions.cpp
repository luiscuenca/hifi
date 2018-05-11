//
//  CharacterDetailedCollisions.cpp
//  libraries/physics/src
//
//  Created by Luis Cuenca 5/11/2018
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "CharacterDetailedCollisions.h"
#include "PhysicsCollisionGroups.h"


class ClosestNotMyBodies : public btCollisionWorld::AllHitsRayResultCallback {
public:
    ClosestNotMyBodies(const CharacterDetailedCollisions* me, const QVector<uint>& jointsFilter)
        : btCollisionWorld::AllHitsRayResultCallback(btVector3(0.0f, 0.0f, 0.0f), btVector3(0.0f, 0.0f, 0.0f)) {
        _me = me;
        _filter = jointsFilter;
        // the RayResultCallback's group and mask must match MY_AVATAR
        m_collisionFilterGroup = BULLET_COLLISION_GROUP_DETAILED_RAY;
        m_collisionFilterMask = BULLET_COLLISION_MASK_DETAILED_RAY;
    }

    virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) override {

        auto rigidBody = rayResult.m_collisionObject;
        auto detailedBodies = _me->getRigidBodies();
        auto hit = std::find_if(detailedBodies.begin(), detailedBodies.end(), [rigidBody](const CharacterDetailedCollisions::CharacterDetailedRigidBody& detailedBody) {
            return  detailedBody._rigidBody == rigidBody;
        });
        auto index = std::distance(detailedBodies.begin(), hit);
        if (hit == detailedBodies.end() || std::find(_filter.begin(), _filter.end(), index) != _filter.end()) {
            return 1.0f;
        }
        return AllHitsRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
    }
protected:
    const CharacterDetailedCollisions* _me;
    QVector<uint> _filter;
};

CharacterDetailedCollisions::CharacterDetailedRigidBody::CharacterDetailedRigidBody(std::vector<btVector3>& shapePoints) {
    btConvexHullShape* shape = new btConvexHullShape(reinterpret_cast<btScalar*>(shapePoints.data()), (int)shapePoints.size());
    shape->setMargin(DETAILED_COLLISION_RADIUS);
    //_motionState = new btDefaultMotionState();
    _rigidBody = new btRigidBody(0.0f, _motionState, shape);
    _rigidBody->setCollisionShape(shape);
    //_rigidBody->setCollisionFlags(btCollisionObject::CollisionFlags::CF_CHARACTER_OBJECT);
    _rigidBody->setActivationState(DISABLE_DEACTIVATION);
    _rigidBody->setSleepingThresholds(0.0, 0.0);
}

CharacterDetailedCollisions::CharacterDetailedRigidBody::CharacterDetailedRigidBody(btVector3& bbox, btVector3& offset) {
    auto x = bbox.getX();
    auto y = bbox.getY();
    auto z = bbox.getZ();
    float smallest = std::min(x, std::min(y, z));
    float margin = smallest*0.4f;
    _offset = offset;
    btBoxShape* box = new btBoxShape(btVector3(x - margin, y - margin, z - margin));
    box->setMargin(margin);
    _motionState = new btDefaultMotionState();
    _rigidBody = new btRigidBody(DETAILED_MASS_KINEMATIC, _motionState, box);
    _rigidBody->setCollisionShape(box);
    _rigidBody->setCollisionFlags(btCollisionObject::CollisionFlags::CF_CHARACTER_OBJECT);
    _rigidBody->setActivationState(DISABLE_DEACTIVATION);
    _rigidBody->setSleepingThresholds(0.0, 0.0);
}

void CharacterDetailedCollisions::CharacterDetailedRigidBody::cleanCollision() {
    if (_motionState) {
        delete _motionState;
        _motionState = nullptr;
    }
    if (_rigidBody) {
        btCollisionShape* shape = _rigidBody->getCollisionShape();
        if (shape) {
            delete shape;
        }
        delete _rigidBody;
        _rigidBody = nullptr;
    }
}


void CharacterDetailedCollisions::CharacterDetailedRigidBody::setTransform(float deltaTime, btTransform& transform) {
    _rotation = transform.getRotation();
    _position = transform.getOrigin() + _offset.rotate(_rotation.getAxis(), _rotation.getAngle());
    btTransform lastTransform = _rigidBody->getWorldTransform();
    btVector3 velocity = transform.getOrigin() - lastTransform.getOrigin();
    float invDeltaTime = 1.0f / deltaTime;
    btVector3 linearVelocity = invDeltaTime * velocity;

    glm::quat orientation = bulletToGLM(transform.getRotation());
    glm::quat lastOrientation = bulletToGLM(lastTransform.getRotation());
    glm::quat delta = glm::inverse(lastOrientation) * orientation;
    glm::vec3 angularVelocity = glm::axis(delta) * glm::angle(delta) * invDeltaTime;

    // _rigidBody->setAngularVelocity(glmToBullet(angularVelocity));
    // _rigidBody->setLinearVelocity(linearVelocity);
    _rigidBody->setWorldTransform(transform);
}

void CharacterDetailedCollisions::setDynamicsWorld(btDynamicsWorld* world) {
    _world = world;
    if (!_updated) {
        updateCollisions();
    }
}

bool CharacterDetailedCollisions::hasRigidBody(int jointIndex) {
    return _rigidBodies.size() > jointIndex && _rigidBodies[jointIndex]._rigidBody;
}

void CharacterDetailedCollisions::setRigidBodyTransform(float deltaTime, int jointIndex, glm::quat& rotation, glm::vec3& position) {
    if (hasRigidBody(jointIndex)) {
        btTransform transform = btTransform(glmToBullet(rotation), glmToBullet(position));
        _rigidBodies[jointIndex].setTransform(deltaTime, transform);
    }
}

void CharacterDetailedCollisions::setRigidBodyTransform(float deltaTime, int jointIndex, btTransform& transform) {
    if (hasRigidBody(jointIndex)) {
        _rigidBodies[jointIndex].setTransform(deltaTime, transform);
    }
}

void CharacterDetailedCollisions::cleanCollisions() {
    for (int i = 0; i < (int)_rigidBodies.size(); i++) {
        _rigidBodies[i].cleanCollision();
    }
    _rigidBodies.clear();
}


void CharacterDetailedCollisions::removeCollisions() {
    if (_world) {
        for (int i = 0; i < (int)_rigidBodies.size(); i++) {
            if (hasRigidBody(i)) {
                _world->removeCollisionObject(_rigidBodies[i]._rigidBody);
            }
        }
        _updated = false;
    }
}

void CharacterDetailedCollisions::updateCollisions() {
    if (_world) {
        for (int i = 0; i < (int)_rigidBodies.size(); i++) {
            if (hasRigidBody(i)) {
                _world->addCollisionObject(_rigidBodies[i]._rigidBody, BULLET_COLLISION_GROUP_DETAILED, BULLET_COLLISION_MASK_DETAILED);
            }
        }
        _updated = true;
    }
}

void CharacterDetailedCollisions::addRigidBody(std::vector<btVector3>& points) {
    if (points.size() > 3) {
        _rigidBodies.push_back(CharacterDetailedRigidBody(points));
    }
    else {
        _rigidBodies.push_back(CharacterDetailedRigidBody());
    }
}

void CharacterDetailedCollisions::addRigidBody(btVector3& bbox, btVector3& offsets) {
    _rigidBodies.push_back(CharacterDetailedRigidBody(bbox, offsets));
}

CharacterDetailedCollisions::RayJointResult CharacterDetailedCollisions::rayTest(const btVector3& origin, const btVector3& direction, const btScalar& length,
    const QVector<uint>& jointsToExclude) const {
    RayJointResult result;
    if (_world && _updated) {

        QVector<btRigidBody*> allBodies;
        for (auto body : _rigidBodies) {
            allBodies.push_back(body._rigidBody);
        }

        ClosestNotMyBodies rayCallback(this, jointsToExclude);
        btVector3 end = origin + length * direction;
        rayCallback.m_closestHitFraction = 1.0f;
        _world->rayTest(origin, end, rayCallback);

        if (rayCallback.m_hitFractions.size() > 0) {
            int minIndex = 0;
            float hitFraction = rayCallback.m_hitFractions[0];
            for (auto i = 1; i < rayCallback.m_hitFractions.size(); i++) {
                if (hitFraction > rayCallback.m_hitFractions[i]) {
                    hitFraction = rayCallback.m_hitFractions[i];
                    minIndex = i;
                }
            }
            for (auto i = 0; i < _rigidBodies.size(); i++) {
                if (_rigidBodies[i]._rigidBody && rayCallback.m_collisionObjects[minIndex] == _rigidBodies[i]._rigidBody) {
                    result._intersectWithJoint = i;
                    result._distance = length * hitFraction;
                    result._intersectionPoint = bulletToGLM(rayCallback.m_hitPointWorld[minIndex]);
                    break;
                }
            }
        }
    }
    return result;
}