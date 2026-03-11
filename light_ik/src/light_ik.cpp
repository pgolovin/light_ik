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

size_t LightIK::CreateIKChain(const std::vector<BoneDesc>& rootChainDesc, int chainStartIndex)
{
    size_t index = m_solvers.size();

    auto& solver = m_solvers.emplace_back(m_skeleton->AddSolver(rootChainDesc, chainStartIndex));
    // TODO: the chain must be formed automatically after creation
    m_skeleton->CompleteChain(solver);

    return index;
}

void LightIK::AddBone(real length, const Quaternion& orientation)
{
    //FIXME:
    // m_solver->AddBone(length, orientation);
    m_relativeRotations.emplace_back(nullptr);
}

void LightIK::CompleteChain()
{

}

void LightIK::SetConstraint(size_t boneIndex, Constraints && constraint)
{
    m_skeleton->SetConstraint(boneIndex, std::move(constraint));
}

void LightIK::SetRootPosition(const Vector& rootPosition)
{

}

void LightIK::SetTargetPosition(size_t chainIndex, const Vector& targetPosition)
{
    assert(chainIndex < m_solvers.size());
    m_solvers[chainIndex].get().SetTargetPosition(targetPosition);
}

size_t LightIK::UpdateChains(size_t iterrations)
{
    size_t count = iterrations;
    for (Solver& solver : m_solvers)
    {
        for (size_t i = 0; i < iterrations; ++i)
        {
            // break the loop if target reached
            if (glm::length2(solver.GetTipPosition() - solver.GetTargetPosition()) < EPSILON)
            {
                // return false if no iterations were done
                count = std::min(count, i);
                break;
            }
            // calculate new rotation of the bone chains
            solver.IterateBack();
            m_skeleton->CompleteChain(solver);
        }
    }
    return count;
}

const std::vector<const Quaternion*> &LightIK::GetDeltaRotations(size_t chainIndex)
{
    assert(chainIndex < m_solvers.size());
    const Solver& solver = m_solvers[chainIndex];
    //FIXME
    const auto& chain = solver.GetChain();

    m_relativeRotations.resize(0);
    for (auto& bone : chain)
    {
        m_relativeRotations.push_back(&bone.get().GetRotation());
    }
    
    return m_relativeRotations;
}


Vector LightIK::GetTargetPosition(size_t chainIndex) const
{
    assert(chainIndex < m_solvers.size());
    return m_solvers[chainIndex].get().GetTargetPosition();
}

real LightIK::GetBoneLength(size_t index) const
{
    assert(index < m_skeleton->GetBones().size()); 
    assert(m_skeleton->GetBones().at(index));

    return m_skeleton->GetBones().at(index)->GetLength();
}

Vector LightIK::GetRootPosition(size_t chainIndex) const
{
    assert(chainIndex < m_solvers.size());
    return m_solvers[chainIndex].get().GetRootPosition();
}

}
 