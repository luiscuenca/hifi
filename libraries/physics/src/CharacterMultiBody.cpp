//
//  CharacterMultiBody.cpp
//  libraries/physics/src
//
//  Created by Luis Cuenca 8/4/2018
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "CharacterMultiBody.h"
#include <qjsonobject.h>
#include "RegisteredMetaTypes.h"

void CharacterMultiBody::setDynamicsWorld(btMultiBodyDynamicsWorld* world) {
    _world = world;
    if (!_multiUpdated) {
        updateAvatarMultiBody();
    }
}

void CharacterMultiBody::setupMultiBody() {
    //init the base	
    CharacterRigidBody baseBody = _rigidBodies[0];
    
    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = 1.f;

    if (baseMass) {
        btCollisionShape *pTempBox = new btBoxShape(baseBody._boxHalfBounds);
        pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
        delete pTempBox;
    }

    bool canSleep = false;
    bool fixedBase = true;

    _numLinks = _rigidBodies.size() - 1;
    if (_numLinks < 1) {
        return;
    }
    if (_avatarMultiBody == nullptr) {
        _avatarMultiBody = new btMultiBody(_numLinks, baseMass, baseInertiaDiag, fixedBase, canSleep);
    }

    _avatarMultiBody->setBasePos(baseBody._defaultPosition);
    _avatarMultiBody->setWorldToBaseRot(baseBody._defaultRotation);

    //init the links	
    btVector3 hingeJointAxis(1, 0, 0);
    float linkMass = 0.1f;
    btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

    btCollisionShape *pTempBox = new btBoxShape(_rigidBodies[1]._boxHalfBounds);
    pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
    delete pTempBox;

    btScalar q0 = 0.f * SIMD_PI / 180.f;
    btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
    quat0.normalize();
    /////

    for (int i = 0; i < _numLinks; ++i)
    {
        CharacterRigidBody linkBody = _rigidBodies[i + 1];
        CharacterRigidBody prevLinkBody = _rigidBodies[i];

        btVector3 parentComToCurrentCom(0, -(prevLinkBody._boxHalfBounds[1] + linkBody._boxHalfBounds[1]), 0);   //par body's COM to cur body's COM offset	
        btVector3 currentPivotToCurrentCom(0, -linkBody._boxHalfBounds[1], 0);							            //cur body's COM to cur body's PIV offset
        btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	                    //par body's COM to cur body's PIV offset

        switch (linkBody._constraintType) {
        case ConstraintType::Fixed:
            _avatarMultiBody->setupFixed(i, linkMass, linkInertiaDiag, linkBody._parentIndex, linkBody._defaultRotation, parentComToCurrentPivot, currentPivotToCurrentCom, true);
            break;
        case ConstraintType::Planar:
            _avatarMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, linkBody._parentIndex, linkBody._defaultRotation, linkBody._hingeJointAxis, parentComToCurrentPivot, true);
            break;
        case ConstraintType::Prismatic:
            _avatarMultiBody->setupPrismatic(i, linkMass, linkInertiaDiag, linkBody._parentIndex, linkBody._defaultRotation, linkBody._hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
            break;
        case ConstraintType::Revolute:
            _avatarMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, linkBody._parentIndex, linkBody._defaultRotation, linkBody._hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
            break;
        case ConstraintType::Spherical:
            _avatarMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, linkBody._parentIndex, linkBody._defaultRotation, parentComToCurrentPivot, currentPivotToCurrentCom, true);
            break;
        }
    }
    _avatarMultiBody->finalizeMultiDof();

    _avatarMultiBody->setCanSleep(_canSleep);
    _avatarMultiBody->setHasSelfCollision(_selfCollide);
    _avatarMultiBody->setUseGyroTerm(_gyro);
    if (!_damping) {
        _avatarMultiBody->setLinearDamping(0.f);
        _avatarMultiBody->setAngularDamping(0.f);
    } else {
        _avatarMultiBody->setLinearDamping(0.1f);
        _avatarMultiBody->setAngularDamping(0.9f);
    }
    if (_numLinks > 0) {
        btScalar q0 = 45.f * SIMD_PI / 180.f;
        if (!_spherical) {
            _avatarMultiBody->setJointPosMultiDof(0, &q0);
        } else {
            btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
            quat0.normalize();
            _avatarMultiBody->setJointPosMultiDof(0, quat0);
        }
    }
}

CharacterRigidBody CharacterMultiBody::getJointConfiguration(const glm::vec3& translation, const glm::quat& rotation, float startRadius, float endRadius, int parentIndex, float offset) {
    float maxRadius = std::max(startRadius, endRadius);
    float distance = glm::length(translation);
    btVector3 halfExtend(maxRadius, 0.5f * distance, maxRadius);
    std::vector<btVector3> positions;
    std::vector<float> radius;
    if (startRadius != 0.0f) {
        positions.push_back(btVector3(0.0f, 0.5f * distance - offset, 0.0f));
        radius.push_back(startRadius);
    }
    if (endRadius != 0.0f) {
        positions.push_back(btVector3(0.0f, -0.5f * distance, 0.0f));
        radius.push_back(endRadius);
    }
    CharacterRigidBody body = CharacterRigidBody();
    body._parentIndex = parentIndex;
    body._boxHalfBounds = halfExtend;
    body._defaultPosition = glmToBullet(translation);
    body._defaultRotation = glmToBullet(rotation);
    body._bodyType = BodyType::Sphere;
    body._spheresPositions = positions;
    body._spheresRadius = radius;
    body._constraintType = ConstraintType::Spherical;
    return body;
}

glm::quat getQuatFromEuler(float eulerX, float eulerY, float eulerZ) {
    glm::quat rotationOffset = (glm::angleAxis(eulerX * RADIANS_PER_DEGREE, Vectors::UNIT_X) *
        glm::angleAxis(eulerY * RADIANS_PER_DEGREE, Vectors::UNIT_Y) *
        glm::angleAxis(eulerZ * RADIANS_PER_DEGREE, Vectors::UNIT_Z));
    return rotationOffset;
}

void CharacterMultiBody::createAvatarMultiBody(const QVariantList& skeleton) {
    
    if (_avatarMultiBody) {
        cleanAvatarMultiBody();
    }

    QVector<bool> isValidJoint;
    _newIndexes.clear();

    for (auto var : skeleton) {
        auto jointInfo = var.toJsonObject();
        
        QString name = jointInfo["name"].toString();
        int index = jointInfo["index"].toInt();
        int parentIndex = jointInfo["parentIndex"].toInt();
        bool isValid = (name == "Hips") ? true : isValidJoint[parentIndex] && !name.contains("Hand");

        isValidJoint.push_back(isValid);

        auto rotation = quatFromVariant(jointInfo["jointRotation"].toVariant());
        auto translation = vec3FromVariant(jointInfo["jointTranslation"].toVariant()) * 0.01f;
        auto delta = rotationBetween(glm::vec3(0.0f, 1.0f, 0.0f), translation);
        rotation = delta * rotation;
        translation = glm::vec3(0.0f, glm::length(translation), 0.0f);

        
        _newIndexes.push_back(isValid ? _rigidBodies.size() - 1 : -1);
        if (isValid) {
            CharacterRigidBody body = getJointConfiguration(translation, rotation, 0.1f, 0.1f, _newIndexes[parentIndex], 0.12f);
            qDebug() << name << " " << _newIndexes[index] << " " << _newIndexes[parentIndex] << " " << vec3toVariant(translation) << " " << quatToVariant(rotation);
            _rigidBodies.push_back(body);
            if (name != "Hips") {
                _constraints.push_back(0);
            }
        }       
    }
    /*
    
    glm::quat rot1 = getQuatFromEuler(15.0f, 0.0f, -15.0f);
    glm::quat rot2 = getQuatFromEuler(15.0f, 0.0f, 15.0f);
    CharacterRigidBody baseBody = getJointConfiguration(glm::vec3(0.0f, 0.3f, 0.0f), rot1, 0.03f, 0.03f, -1);

    CharacterRigidBody linkBody01 = getJointConfiguration(glm::vec3(0.0f, 0.6f, 0.0f), rot1, 0.03f, 0.05f, -1);
    CharacterRigidBody linkBody02 = getJointConfiguration(glm::vec3(0.0f, 0.3f, 0.0f), rot1, 0.05f, 0.05f, 0);
    CharacterRigidBody linkBody03 = getJointConfiguration(glm::vec3(0.0f, 0.6f, 0.0f), rot1, 0.05f, 0.08f, 1);
    CharacterRigidBody linkBody04 = getJointConfiguration(glm::vec3(0.0f, 0.3f, 0.0f), rot1, 0.08f, 0.05f, 2);
    CharacterRigidBody linkBody05 = getJointConfiguration(glm::vec3(0.0f, 0.2f, 0.0f), rot1, 0.05f, 0.05f, 3);

    CharacterRigidBody linkBody06 = getJointConfiguration(glm::vec3(0.0f, 0.6f, 0.0f), rot2, 0.03f, 0.05f, -1);
    CharacterRigidBody linkBody07 = getJointConfiguration(glm::vec3(0.0f, 0.3f, 0.0f), rot2, 0.05f, 0.05f, 5);
    CharacterRigidBody linkBody08 = getJointConfiguration(glm::vec3(0.0f, 0.6f, 0.0f), rot2, 0.05f, 0.08f, 6);
    CharacterRigidBody linkBody09 = getJointConfiguration(glm::vec3(0.0f, 0.3f, 0.0f), rot2, 0.08f, 0.05f, 7);
    CharacterRigidBody linkBody10 = getJointConfiguration(glm::vec3(0.0f, 0.2f, 0.0f), rot2, 0.05f, 0.05f, 8);

    _rigidBodies.push_back(baseBody);
    _rigidBodies.push_back(linkBody01);
    _rigidBodies.push_back(linkBody02);
    _rigidBodies.push_back(linkBody03);
    _rigidBodies.push_back(linkBody04);
    _rigidBodies.push_back(linkBody05);
    _rigidBodies.push_back(linkBody06);
    _rigidBodies.push_back(linkBody07);
    _rigidBodies.push_back(linkBody08);
    _rigidBodies.push_back(linkBody09);
    _rigidBodies.push_back(linkBody10);
    */

    setupMultiBody();
    addAvatarMultiBodyColliders();
    _multiUpdated = false;    
}


void CharacterMultiBody::addAvatarMultiBodyColliders()
{
    static float friction = 1.;
    btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(_avatarMultiBody->getNumLinks() + 1);

    btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(_avatarMultiBody->getNumLinks() + 1);
    world_to_local[0] = _avatarMultiBody->getWorldToBaseRot();
    local_origin[0] = _avatarMultiBody->getBasePos();

    //	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
    btScalar quat[4] = { -world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w() };

    CharacterRigidBody baseBody = _rigidBodies[0];
    btMultiBodyLinkCollider* baseCollider = nullptr;
    if (baseBody._bodyType == BodyType::Box) {
        btCollisionShape* box = new btBoxShape(baseBody._boxHalfBounds);
        baseCollider = new btMultiBodyLinkCollider(_avatarMultiBody, -1);
        baseCollider->setCollisionShape(box);
    } else {
        btMultiSphereShape* mshape = new btMultiSphereShape(baseBody._spheresPositions.data(), baseBody._spheresRadius.data(), baseBody._spheresRadius.size());
        baseCollider = new btMultiBodyLinkCollider(_avatarMultiBody, -1);
        baseCollider->setCollisionShape(mshape);
    }
    
    //baseCollider->setCollisionFlags(BULLET_COLLISION_GROUP_KINEMATIC);

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(local_origin[0]);
    tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
    baseCollider->setWorldTransform(tr);
    baseCollider->setFriction(friction);
    _rigidBodies[0]._body = baseCollider;
    _avatarMultiBody->setBaseCollider(baseCollider);


    for (int i = 0; i < _avatarMultiBody->getNumLinks(); ++i) {
        const int parent = _avatarMultiBody->getParent(i);
        world_to_local[i + 1] = _avatarMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
        local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), _avatarMultiBody->getRVector(i)));
    }


    for (int i = 0; i < _avatarMultiBody->getNumLinks(); ++i) {

        btVector3 posr = local_origin[i + 1];
        //	float pos[4]={posr.x(),posr.y(),posr.z(),1};

        btScalar quat[4] = { -world_to_local[i + 1].x(),-world_to_local[i + 1].y(),-world_to_local[i + 1].z(),world_to_local[i + 1].w() };

        CharacterRigidBody linkBody = _rigidBodies[i + 1];
        btMultiBodyLinkCollider* linkCollider = nullptr;
        if (linkBody._bodyType == BodyType::Box) {
            btCollisionShape* box = new btBoxShape(linkBody._boxHalfBounds);
            linkCollider = new btMultiBodyLinkCollider(_avatarMultiBody, i);
            linkCollider->setCollisionShape(box);
        } else {
            btMultiSphereShape* mshape = new btMultiSphereShape(linkBody._spheresPositions.data(), linkBody._spheresRadius.data(), linkBody._spheresRadius.size());
            linkCollider = new btMultiBodyLinkCollider(_avatarMultiBody, i);
            linkCollider->setCollisionShape(mshape);
        }
        //linkCollider->setCollisionFlags(BULLET_COLLISION_GROUP_KINEMATIC);
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
        linkCollider->setWorldTransform(tr);
        linkCollider->setFriction(friction);
        _rigidBodies[i + 1]._body = linkCollider;
        _avatarMultiBody->getLink(i).m_collider = linkCollider;
    }
}

void CharacterMultiBody::removeAvatarMultiBody() {
    if (_world) {
        if (_avatarMultiBody) {
            for (int i = 0; i < _rigidBodies.size(); i++) {
                _world->removeCollisionObject(_rigidBodies[i]._body);
            }
            _world->removeMultiBody(_avatarMultiBody);
        }
    }
    _multiUpdated = false;
}

void CharacterMultiBody::cleanAvatarMultiBody() {
    removeAvatarMultiBody();
    if (_world && _avatarMultiBody) {
        for (int i = 0; i < _rigidBodies.size(); i++) {
            delete _rigidBodies[i]._body;
            _rigidBodies[i]._body = nullptr;
        }
        for (int i = 0; i < _constraints.size(); i++) {
            destroyConstraint(i);
        }
        _rigidBodies.clear();
        delete _avatarMultiBody;
        _avatarMultiBody = nullptr;
    }
}

void CharacterMultiBody::updateConstraint(int linkIndex, const glm::vec3& position) {
    btMultiBodyPoint2Point* constraint = _constraints[linkIndex];
    btVector3 pivotInB = glmToBullet(position);
    btVector3 pivotInA = _avatarMultiBody->worldPosToLocal(linkIndex, pivotInB);
    if (constraint) {
        constraint->setPivotInB(pivotInB);
        _constraints[linkIndex]->setMaxAppliedImpulse(linkIndex == _allowUpdateIndex ? 0.5f : 0.0f);
    } else {
        _constraints[linkIndex] = new btMultiBodyPoint2Point(_avatarMultiBody, linkIndex, 0, pivotInA, pivotInB);
        _constraints[linkIndex]->setMaxAppliedImpulse(0.0f);
        _world->addMultiBodyConstraint(_constraints[linkIndex]);
    }

}

void CharacterMultiBody::destroyConstraint(int linkIndex) {
    btMultiBodyPoint2Point* constraint = _constraints[linkIndex];
    if (constraint) {
        btMultiBodyPoint2Point* constraint = _constraints[linkIndex];
        if (constraint) {
            _world->removeMultiBodyConstraint(constraint);
            delete constraint;
            _constraints[linkIndex] = 0;
        }
    }
}

void CharacterMultiBody::updateAvatarMultiBody() {
    if (_world) {
        if (!_avatarMultiBody && _skeleton.size() > 0) {
            createAvatarMultiBody(_skeleton);
        }
        _world->addMultiBody(_avatarMultiBody, _group, _mask);
        _world->setGravity(btVector3(0.f, -9.8f, 0.f));
        for (int i = 0; i < _rigidBodies.size(); i++) {
            _world->addCollisionObject(_rigidBodies[i]._body, _group, _mask);
        }
    }
    _multiUpdated = true;
}

void CharacterMultiBody::setAvatarMultiBodyPosition(float deltaTime, const glm::vec3& newPosition) {
    if (_avatarMultiBody) {// && !_setted) {
        auto transform = _avatarMultiBody->getBaseWorldTransform();
        transform.setOrigin(glmToBullet(newPosition));
        _avatarMultiBody->setBaseWorldTransform(transform);
        _setted = true;
    }
}

void CharacterMultiBody::setAvatarMultiBodyForces(float deltaTime, const QVector<glm::vec3>& jointPositions) {
    if (_avatarMultiBody) {
        _allowUpdateIndex = _allowUpdateIndex + 1 >= _rigidBodies.size() ? 1 : _allowUpdateIndex + 1;
        for (int i = 0; i < jointPositions.size(); i++) {
            auto position = jointPositions[i];
            auto index = _newIndexes[i];
            if (index > -1) {
                updateConstraint(index, position);
            }
        }
    }
}