/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "skeleton.h"
#include "helpers.h"
#include "light_ik/light_ik.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{

LightIK::LightIK(size_t bonesCount)
    : m_skeleton(std::make_unique<Skeleton>(bonesCount))
{
    m_relativeRotations.resize(bonesCount);
}

LightIK::~LightIK()
{
}

void LightIK::ResetPose()
{
    m_skeleton->ResetPose();
}

void LightIK::Reset()
{
    m_solvers.clear();
    m_targets.clear();
    m_skeleton->ResetIK();
}

size_t LightIK::CreateIKChain(const std::vector<BoneDesc>& rootChainDesc, int chainStartIndex, Target& target)
{
    size_t index = m_solvers.size();
    m_solvers.emplace_back(m_skeleton->AddSolver(rootChainDesc, chainStartIndex, target));
    for (const BoneDesc& desc : rootChainDesc)
    {
        Bone* bone = m_skeleton->GetBones()[desc.boneIndex].get();
        m_relativeRotations[desc.boneIndex] = bone ? &bone->GetRotation() : nullptr;
    }
    return index;
}

size_t LightIK::CreateIKLink(const std::vector<BoneDesc>& rootChainDesc, int chainStartIndex, int targetBoneIndex)
{
    size_t index = m_solvers.size();
    std::unique_ptr<TargetBone> bone = std::make_unique<TargetBone>(*m_skeleton);
    bone->AssignBone(targetBoneIndex);
    m_solvers.emplace_back(m_skeleton->AddSolver(rootChainDesc, chainStartIndex, *bone));
    m_targets.emplace_back(std::move(bone));

    for (const BoneDesc& desc : rootChainDesc)
    {
        Bone* bone = m_skeleton->GetBones()[desc.boneIndex].get();
        m_relativeRotations[desc.boneIndex] = bone ? &bone->GetRotation() : nullptr;
    }
    return index;
}

void LightIK::CreatePassiveChain(const std::vector<BoneDesc>& rootChainDesc)
{
    SolverBase* passiveChain = m_skeleton->AddChain(rootChainDesc);
    if (passiveChain)
    {
        m_solvers.emplace_back(*passiveChain);

        for (const BoneDesc& desc : rootChainDesc)
        {
            Bone* bone = m_skeleton->GetBones()[desc.boneIndex].get();
            m_relativeRotations[desc.boneIndex] = bone ? &bone->GetRotation() : nullptr;
        }
    }
}

void LightIK::SetConstraint(size_t boneIndex, Constraints && constraint)
{
    m_skeleton->SetConstraint(boneIndex, std::move(constraint));
}

size_t LightIK::Update(size_t iterations)
{
    return m_skeleton->Update(iterations);
}

const std::vector<const Quaternion*> &LightIK::GetDeltaRotations()
{
    return m_relativeRotations;
}

const Vector& LightIK::GetTargetPosition(size_t chainIndex) const
{
    assert(m_solvers.size() > chainIndex);
    return m_solvers[chainIndex].get().GetTargetPosition();
}

TargetBone& LightIK::CreateInternalTarget()
{
    auto target = std::make_unique<TargetBone>(*m_skeleton);
    TargetBone& ref = *target;
    m_targets.emplace_back(std::move(target));
    return ref;
}

TargetPosition& LightIK::CreateTarget()
{
    auto target = std::make_unique<TargetPosition>();
    TargetPosition& ref = *target;
    m_targets.emplace_back(std::move(target));
    return ref;
}

size_t LightIK::GetSolversCount() const
{
    return m_skeleton->GetSolversCount();
}

Vector LightIK::GetTipPosition(size_t chainIndex) const
{
    assert(chainIndex < m_solvers.size());
    return m_solvers[chainIndex].get().GetTipPosition();
}

real LightIK::GetBoneLength(size_t index) const
{
    assert(index < m_skeleton->GetBones().size()); 
    assert(m_skeleton->GetBones().at(index));

    return m_skeleton->GetBones().at(index)->GetLength();
}

Vector LightIK::GetBonePosition(size_t index) const
{
    assert(index < m_skeleton->GetBones().size()); 
    assert(m_skeleton->GetBones().at(index));

    return m_skeleton->GetBones().at(index)->GetPosition();
}

}
 