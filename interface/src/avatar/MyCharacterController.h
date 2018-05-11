//
//  MyCharacterController.h
//  interface/src/avatar
//
//  Created by AndrewMeadows 2015.10.21
//  Copyright 2015 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//


#ifndef hifi_MyCharacterController_h
#define hifi_MyCharacterController_h

#include <CharacterController.h>
//#include <SharedUtil.h>

class btCollisionShape;
class MyAvatar;

class MyCharacterController : public CharacterController {
public:
    explicit MyCharacterController(MyAvatar* avatar);
    ~MyCharacterController ();

    void setDynamicsWorld(btDynamicsWorld* world) override;
    void updateShapeIfNecessary() override;
    void updateDetailedCollisions(float deltaTime);

    // Sweeping a convex shape through the physics simulation can be expensive when the obstacles are too
    // complex (e.g. small 20k triangle static mesh) so instead we cast several rays forward and if they
    // don't hit anything we consider it a clean sweep.  Hence this "Shotgun" code.
    class RayShotgunResult {
    public:
        void reset();
        float hitFraction { 1.0f };
        bool walkable { true };
    };

    /// return true if RayShotgun hits anything
    bool testRayShotgun(const glm::vec3& position, const glm::vec3& step, RayShotgunResult& result);

    void setDensity(btScalar density) { _density = density; }
    std::vector<std::vector<glm::vec3>> getWorldCollisionShapes() const { return _worldCollisionShapes; };
    std::vector<btTransform> getWorldCollisionTransforms() const;

    bool isInPhysicsSimulation(QUuid avatarId);
    void addOtherAvatarDetailedCollisions(QUuid avatarId, std::vector<std::vector<btVector3>>& shapes);
    void addOtherAvatarDetailedCollisions(QUuid avatarId, std::vector<btVector3>& bboxes, std::vector<btVector3>& offsets);
    void removeOtherAvatarDetailedCollisions(QUuid avatarId);
    void updateOtherAvatarDetailedCollisons(float deltaTime, QUuid avatarId, std::vector<btTransform>& transforms);
    const CharacterDetailedCollisions& getAvatarDetailedCollisions(QUuid avatarId) const { return _otherCharactersDetailedCollisions.find(avatarId)->second; };
    const CharacterDetailedCollisions& getMyAvatarDetailedCollisions() const { return _detailedCollisions; };
protected:
    void initRayShotgun(const btCollisionWorld* world);
    void updateMassProperties() override;

private:
    btConvexHullShape* computeShape() const;
    void updateDetailedCollisionsShapes();
    std::vector<std::vector<glm::vec3>> _worldCollisionShapes;

protected:
    MyAvatar* _avatar { nullptr };

    // shotgun scan data
    btAlignedObjectArray<btVector3> _topPoints;
    btAlignedObjectArray<btVector3> _bottomPoints;
    btScalar _density { 1.0f };
};

#endif // hifi_MyCharacterController_h
