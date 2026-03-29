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

void LightIK::Reset()
{
    //FIXME
    // Reset solver and all bones;
    // m_solver = std::make_unique<Solver>();
}

size_t LightIK::CreateIKChain(const std::vector<BoneDesc>& rootChainDesc, int chainStartIndex, Target& target)
{
    size_t index = m_solvers.size();
    m_solvers.emplace_back(m_skeleton->AddSolver(rootChainDesc, chainStartIndex, target));
    for (const BoneDesc& desc : rootChainDesc)
    {
        m_relativeRotations[desc.boneIndex] = &m_skeleton->GetBones()[desc.boneIndex]->GetRotation();
    }
    return index;
}

size_t LightIK::CreateChain(const std::vector<BoneDesc>& rootChainDesc)
{
    size_t index = m_solvers.size();
    m_solvers.emplace_back(m_skeleton->AddChain(rootChainDesc));
    for (const BoneDesc& desc : rootChainDesc)
    {
        m_relativeRotations[desc.boneIndex] = &m_skeleton->GetBones()[desc.boneIndex]->GetRotation();
    }
    return index;
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

TargetBone LightIK::CreateInternalTarget() const
{
    return TargetBone(*m_skeleton);
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
 