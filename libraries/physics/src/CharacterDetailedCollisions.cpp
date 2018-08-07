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
        m_collisionFilterGroup = BULLET_COLLISION_GROUP_DYNAMIC;
        m_collisionFilterMask = BULLET_COLLISION_MASK_DYNAMIC;
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

CharacterDetailedCollisions::CharacterDetailedRigidBody::CharacterDetailedRigidBody(std::vector<btVector3>& shapePoints, int type) {
    btConvexHullShape* shape = new btConvexHullShape(reinterpret_cast<btScalar*>(shapePoints.data()), (int)shapePoints.size());
    _config = CharacterDetailedConfig();
    shape->setMargin(DETAILED_COLLISION_RADIUS);
    // _motionState = new btDefaultMotionState();
    _type = type;
    float mass = type::KINEMATIC ? DETAILED_MASS_KINEMATIC : DETAILED_MASS_DYNAMIC;
    _rigidBody = new btRigidBody(DETAILED_MASS_KINEMATIC, _motionState, shape);
    _rigidBody->setCollisionShape(shape);
    _rigidBody->setCollisionFlags(_rigidBody->getCollisionFlags() & ~(btCollisionObject::CF_KINEMATIC_OBJECT |
                                                                        btCollisionObject::CF_STATIC_OBJECT));
    _rigidBody->setActivationState(DISABLE_DEACTIVATION);
    _rigidBody->setSleepingThresholds(0.0, 0.0);
    _rigidBody->setFlags(BT_DISABLE_WORLD_GRAVITY);

    qWarning() << "Creating new rigid body";
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
    _rigidBody->setFlags(BT_DISABLE_WORLD_GRAVITY);
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

    const int FORCE_POSITION_FRAME_COUNT = 180;
        
    btVector3 deltaFrame = transform.getOrigin() - _lastTransform.getOrigin();
    btVector3 deltaPosition = transform.getOrigin() - _rigidBody->getWorldTransform().getOrigin();
    float invDeltaTime = 1.0f / deltaTime;
    auto deltaFrameStep = invDeltaTime * deltaFrame;
    auto deltaPositionStep = invDeltaTime * deltaPosition;

    auto lastRotation = _rigidBody->getWorldTransform().getRotation();
    auto targetRotation = transform.getRotation();
    auto deltaRotation = targetRotation * lastRotation.inverse();
   
    if (_init < FORCE_POSITION_FRAME_COUNT) {
        _lastTransform = _rigidBody->getWorldTransform();
        _init++;
        _rigidBody->setWorldTransform(transform);
        _lastDeltaRotation = deltaRotation;
        return;
    }
    if (deltaRotation.dot(_lastDeltaRotation) < 0) {
        deltaRotation = -deltaRotation;
    }
    auto axis = deltaRotation.getAxis();
    auto angle = deltaRotation.getAngle();
    btVector3 angularVelocity(axis * angle * invDeltaTime);

    _lastDeltaRotation = deltaRotation;
    _rigidBody->setAngularVelocity(angularVelocity);

    float finalAttenuation = (_config._attenuate && deltaFrame.length() < _config._attenuationThreshold) ? _config._attenuationValue : 1.0f;

    if (_config._applyLinearVelocity) {
        if (_config._velocityDeltaType == _config.delta::FRAME) {
            _rigidBody->setLinearVelocity(finalAttenuation*_config._velocityDeltaFrameMult*deltaFrameStep);
        } else {
            _rigidBody->setLinearVelocity(finalAttenuation*_config._velocityDeltaPositionMult*deltaPositionStep);
        }
    }

    if (_config._applyImpulse) {
        if (_config._impulseDeltaType == _config.delta::FRAME) {
            _rigidBody->applyCentralImpulse(finalAttenuation*_config._impulseDeltaFrameMult*deltaFrameStep);
        } else {
            _rigidBody->applyCentralImpulse(finalAttenuation*_config._impulseDeltaPositionMult*deltaPositionStep);
        }
    }

    if (_config._applyForce) {
        if (_config._forceDeltaFrameMult == _config.delta::FRAME) {
            _rigidBody->applyCentralForce(finalAttenuation*_config._forceDeltaFrameMult*deltaFrameStep);
        } else {
            _rigidBody->applyCentralForce(finalAttenuation*_config._forceDeltaPositionMult*deltaPositionStep);
        }
    }
    _lastTransform = transform;
}

void CharacterDetailedCollisions::setDynamicsWorld(btMultiBodyDynamicsWorld* world) {
    _world = world;
    if (!_updated) {
        updateCollisions();
    }
    if (!_multiUpdated) {
        updateAvatarMultiBody();
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
                _world->addRigidBody(_rigidBodies[i]._rigidBody, _group, _mask);
            }
        }
        _updated = true;
    }
}

void CharacterDetailedCollisions::addRigidBody(std::vector<btVector3>& points, int type) {
    if (points.size() > 3) {
        _rigidBodies.push_back(CharacterDetailedRigidBody(points, type));
    }
    else {
        _rigidBodies.push_back(CharacterDetailedRigidBody());
    }
}

void CharacterDetailedCollisions::addRigidBody(btVector3& bbox, btVector3& offsets) {
    _rigidBodies.push_back(CharacterDetailedRigidBody(bbox, offsets));
}

void CharacterDetailedCollisions::getPhysicsConfig(CharacterDetailedCollisions::CharacterDetailedConfig& config) {
    for (int i = 0; i < _rigidBodies.size(); i++) {
        if (_rigidBodies[i]._rigidBody != NULL) {
            config = _rigidBodies[i].getConfig();
            return;
        }
    }
}

void CharacterDetailedCollisions::setPhysicsConfig(const CharacterDetailedCollisions::CharacterDetailedConfig& config) {
    for (int i = 0; i < _rigidBodies.size(); i++) {
        if (hasRigidBody(i)) {
            _rigidBodies[i].setConfig(config);
        }
    }
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

btMultiBody* CharacterDetailedCollisions::createFeatherstoneMultiBody(int numLinks, const btVector3 &basePosition, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents, bool spherical, bool floating)
{
    //init the base	
    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = 1.f;

    if (baseMass)
    {
        btCollisionShape *pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
        pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
        delete pTempBox;
    }

    bool canSleep = false;

    btMultiBody *pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);

    btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
    pMultiBody->setBasePos(basePosition);
    pMultiBody->setWorldToBaseRot(baseOriQuat);
    btVector3 vel(0, 0, 0);
    //	pMultiBody->setBaseVel(vel);

    //init the links	
    btVector3 hingeJointAxis(1, 0, 0);
    float linkMass = 1.f;
    btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

    btCollisionShape *pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
    pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
    delete pTempBox;

    //y-axis assumed up
    btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);						//par body's COM to cur body's COM offset	
    btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);							//cur body's COM to cur body's PIV offset
    btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset

                                                                                            //////
    btScalar q0 = 0.f * SIMD_PI / 180.f;
    btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
    quat0.normalize();
    /////

    for (int i = 0; i < numLinks; ++i)
    {
        if (!spherical)
            pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
        else
            //pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
            pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
    }

    pMultiBody->finalizeMultiDof();
    return pMultiBody;
}

void CharacterDetailedCollisions::createAvatarMultiBody() {

    if (_avatarMultiBody) {
        removeAvatarMultiBody();
        cleanAvatarMultiBody();
    }
    bool damping = true;
    bool gyro = true;
    int numLinks = 5;
    bool spherical = true;					//set it ot false -to use 1DoF hinges instead of 3DoF sphericals		
    bool multibodyOnly = false;
    bool canSleep = false;
    bool selfCollide = true;
    bool multibodyConstraint = false;
    btVector3 linkHalfExtents(0.05f, 0.37f, 0.1f);
    btVector3 baseHalfExtents(0.05f, 0.37f, 0.1f);

    _avatarMultiBody = createFeatherstoneMultiBody(numLinks, btVector3(-0.4f, 3.f, 0.f), linkHalfExtents, baseHalfExtents, spherical, false);
    _avatarMultiBody->setCanSleep(canSleep);
    _avatarMultiBody->setHasSelfCollision(selfCollide);
    _avatarMultiBody->setUseGyroTerm(gyro);
    if (!damping)
    {
        _avatarMultiBody->setLinearDamping(0.f);
        _avatarMultiBody->setAngularDamping(0.f);
    }
    else
    {
        _avatarMultiBody->setLinearDamping(0.1f);
        _avatarMultiBody->setAngularDamping(0.9f);
    }
    if (numLinks > 0)
    {
        btScalar q0 = 45.f * SIMD_PI / 180.f;
        if (!spherical)
        {
            _avatarMultiBody->setJointPosMultiDof(0, &q0);
        }
        else
        {
            btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
            quat0.normalize();
            _avatarMultiBody->setJointPosMultiDof(0, quat0);
        }
    }
    addAvatarMultiBodyColliders(baseHalfExtents, linkHalfExtents);
    _multiUpdated = false;
}

void CharacterDetailedCollisions::addAvatarMultiBodyColliders(const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents)
{
    static float friction = 1.;
    btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(_avatarMultiBody->getNumLinks() + 1);

    btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(_avatarMultiBody->getNumLinks() + 1);
    world_to_local[0] = _avatarMultiBody->getWorldToBaseRot();
    local_origin[0] = _avatarMultiBody->getBasePos();

    const std::vector<float> radious = { 0.1f, 0.1f };
    const std::vector<btVector3> positions = { btVector3(0.0f, 0.3f, 0.0f), btVector3(0.0f, -0.3f, 0.0f) };

    {

        //	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
        btScalar quat[4] = { -world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w() };


        if (1)
        {
            // btCollisionShape* box = new btBoxShape(baseHalfExtents);
            btMultiSphereShape* mshape = new btMultiSphereShape(positions.data(), radious.data(), radious.size());
            btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(_avatarMultiBody, -1);
            col->setCollisionShape(mshape);

            btTransform tr;
            tr.setIdentity();
            tr.setOrigin(local_origin[0]);
            tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
            col->setWorldTransform(tr);
            col->setFriction(friction);
            _multiBodies.push_back(col);
            _avatarMultiBody->setBaseCollider(col);
        }
    }


    for (int i = 0; i < _avatarMultiBody->getNumLinks(); ++i)
    {
        const int parent = _avatarMultiBody->getParent(i);
        world_to_local[i + 1] = _avatarMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
        local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), _avatarMultiBody->getRVector(i)));
    }


    for (int i = 0; i < _avatarMultiBody->getNumLinks(); ++i)
    {

        btVector3 posr = local_origin[i + 1];
        //	float pos[4]={posr.x(),posr.y(),posr.z(),1};

        btScalar quat[4] = { -world_to_local[i + 1].x(),-world_to_local[i + 1].y(),-world_to_local[i + 1].z(),world_to_local[i + 1].w() };

        // btCollisionShape* box = new btBoxShape(linkHalfExtents);
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(_avatarMultiBody, i);
        btMultiSphereShape* mshape = new btMultiSphereShape(positions.data(), radious.data(), radious.size());

        col->setCollisionShape(mshape);
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
        col->setWorldTransform(tr);
        col->setFriction(friction);
        _multiBodies.push_back(col);

        _avatarMultiBody->getLink(i).m_collider = col;
    }
}

void CharacterDetailedCollisions::removeAvatarMultiBody() {
    if (_world) {
        if (_avatarMultiBody) {
            for (int i = 0; i < _multiBodies.size(); i++) {
                _world->removeCollisionObject(_multiBodies[i]);
            }
            _world->removeMultiBody(_avatarMultiBody);
        }
    }
    _multiUpdated = false;
}

void CharacterDetailedCollisions::cleanAvatarMultiBody() {
    if (_world) {
        if (_avatarMultiBody) {
            for (int i = 0; i < _multiBodies.size(); i++) {
                delete _multiBodies[i];
            }
            _multiBodies.clear();
            delete _avatarMultiBody;
            _avatarMultiBody = nullptr;
        }
    }
}

void CharacterDetailedCollisions::updateAvatarMultiBody() {
    if (_world) {
        if (!_avatarMultiBody) {
            createAvatarMultiBody();
        }
        _world->addMultiBody(_avatarMultiBody, BULLET_COLLISION_GROUP_DYNAMIC, BULLET_COLLISION_MASK_DYNAMIC);
        _world->setGravity(btVector3(0.f, -9.8f, 0.f));
        for (int i = 0; i < _multiBodies.size(); i++) {
            _world->addCollisionObject(_multiBodies[i], BULLET_COLLISION_GROUP_DYNAMIC, BULLET_COLLISION_MASK_DYNAMIC);
        }
    }
    _multiUpdated = true;
}

void CharacterDetailedCollisions::setAvatarMultiBodyPosition(float deltaTime, const glm::vec3& newPosition) {
    if (_avatarMultiBody) {
        auto transform = _avatarMultiBody->getBaseWorldTransform();
        transform.setOrigin(glmToBullet(newPosition));
        _avatarMultiBody->setBaseWorldTransform(transform);
    }
}