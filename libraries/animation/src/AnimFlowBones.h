//
//  AnimFlowBones.h
//
//  Created by Luis Cuenca on 2/13/19.
//  Copyright (c) 2018 High Fidelity, Inc. All rights reserved.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_AnimFlowBones_h
#define hifi_AnimFlowBones_h

#include "AnimNode.h"
#include "AnimChain.h"

// Three bone IK chain

class AnimFlowBones : public AnimNode {
public:
    friend class AnimTests;

    AnimFlowBones(const QString& id, bool enabled);
    virtual ~AnimFlowBones() override;

    virtual const AnimPoseVec& evaluate(const AnimVariantMap& animVars, const AnimContext& context, float dt, AnimVariantMap& triggersOut) override;

protected:
    // for AnimDebugDraw rendering
    virtual const AnimPoseVec& getPosesInternal() const override;
    virtual void setSkeletonInternal(AnimSkeleton::ConstPointer skeleton) override;

    AnimPoseVec _poses;

    bool _enabled;

    AnimChain _snapshotChain;

    // no copies
    AnimFlowBones(const AnimFlowBones&) = delete;
    AnimFlowBones& operator=(const AnimFlowBones&) = delete;
};

#endif // hifi_AnimFlowBones_h
