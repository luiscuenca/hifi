//
//  AnimFlowBones.h
//
//  Created by Luis Cuenca on 2/13/19.
//  Copyright (c) 2018 High Fidelity, Inc. All rights reserved.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "AnimFlowBones.h"
#include "AnimationLogging.h"
#include "AnimUtil.h"
#include "GLMHelpers.h"

const float FRAMES_PER_SECOND = 30.0f;
const float INTERP_DURATION = 6.0f;

AnimFlowBones::AnimFlowBones(const QString& id, bool enabled) :
    AnimNode(AnimNode::Type::FlowBones, id),
    _enabled(enabled) {
    qDebug() << "Enabled " << enabled;
}

AnimFlowBones::~AnimFlowBones() {

}

const AnimPoseVec& AnimFlowBones::evaluate(const AnimVariantMap& animVars, const AnimContext& context, float dt, AnimVariantMap& triggersOut) {  
    return _children[0]->evaluate(animVars, context, dt, triggersOut);
}

// for AnimDebugDraw rendering
const AnimPoseVec& AnimFlowBones::getPosesInternal() const {
    return _poses;
}

void AnimFlowBones::setSkeletonInternal(AnimSkeleton::ConstPointer skeleton) {
    AnimNode::setSkeletonInternal(skeleton);
}

