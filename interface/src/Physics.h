//
//  Physics.h
//  interface/src
//
//  Created by Philip on 4/25/13.
//  Copyright 2013 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_Physics_h
#define hifi_Physics_h

#include <PhysicsEngine.h>

class EntityTree;

class ThreadSafePhysicsEngine : public PhysicsEngine {
public:
    ThreadSafePhysicsEngine(const glm::vec3& offset);

    // virtual override from PhysicsEngine
    void init() { assert(false); } // call initSafe() instead

    void initSafe(EntityTree* entities);
};

void applyStaticFriction(float deltaTime, glm::vec3& velocity, float maxVelocity, float strength);
void applyDamping(float deltaTime, glm::vec3& velocity, float linearStrength, float squaredStrength);

#endif // hifi_Physics_h
