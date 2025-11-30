/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "solver.h"
#include "helpers.h"
#include "light_ik/light_ik.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{

LightIK::LightIK()
    : m_solver(std::make_unique<Solver>())
{
}

LightIK::~LightIK()
{
    m_solver = nullptr;
}

void LightIK::Reset()
{
    //reset solver and all bones;
    m_solver = std::make_unique<Solver>();
}

void LightIK::AddBone(real length, const Quaternion& orientation)
{
    m_solver->AddBone(length, orientation);
}

void LightIK::CompleteChain()
{
    m_solver->CompleteChain();
}

void LightIK::SetConstraint(size_t boneIndex, Constraints && constraint)
{
    m_solver->SetConstraint(boneIndex, std::move(constraint));
}

void LightIK::SetRootPosition(const Vector& rootPosition)
{
    m_solver->OverrideRootPosition(rootPosition);
}

void LightIK::SetTargetPosition(const Vector& targetPosition)
{
    m_solver->SetTargetPosition(targetPosition);
}

size_t LightIK::UpdateChainPosition(size_t iterrations)
{
    for (size_t i = 0; i < iterrations; ++i)
    {
        // break the loop if target reached
        if (glm::length2(m_solver->GetTipPosition() - m_solver->GetTargetPosition()) < EPSILON)
        {
            // return false if no iterations were done
            return i;
        }
        // calculate new rotation of the bone chains
        m_solver->IterateBack();
        m_solver->IterateFront();
    }
    return iterrations;
}

std::vector<Quaternion> LightIK::GetDeltaRotations() const
{
    const auto& chain = m_solver->GetBones().bones;

    std::vector<Quaternion> result;
    result.reserve(chain.size());

    static Quaternion dummy = glm::identity<Quaternion>(); 
    const Quaternion* previousRotation = &dummy;
    for (auto& bone : chain)
    {
        result.push_back(glm::inverse(*previousRotation) * bone.GetGlobalOrientation());
        previousRotation = &bone.GetGlobalOrientation();
    }
    
    return result;
}


Vector LightIK::GetTargetPosition()const
{
    return m_solver->GetTargetPosition();
}

real LightIK::GetBoneLength(size_t index) const
{
    return m_solver->GetBones(0).bones[index].GetLength();
}

Vector LightIK::GetRootPosition() const
{
    return m_solver->GetRootPosition();
}

}
 