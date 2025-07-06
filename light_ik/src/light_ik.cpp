/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "solver.h"
#include "light_ik/light_ik.h"

#include "glm/gtx/norm.inl"
#include "glm/gtx/vector_angle.inl"
#define GLM_ENABLE_EXPERIMENTAL
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

void LightIK::SetRootPosition(const Vector& rootPosition)
{
    m_solver->OverrideRootPosition(rootPosition);
}

void LightIK::AddBone(const Vector& boneEnd)
{
    m_solver->AddBone(boneEnd);
}

void LightIK::SetTargetPosition(const Vector& targetPosition)
{
    m_solver->SetTargetPosition(targetPosition);
}

bool LightIK::UpdateChainPosition(size_t iterrations)
{
    if (glm::length2(m_solver->GetTipPosition() - m_solver->GetTargetPosition()) < DELTA)
    {
        return false;
    }

    for (size_t i = 0; i < iterrations; ++i)
    {
        m_solver->IterateBack();
        m_solver->IterateFront();
    }
    return true;
}

void LightIK::GetBoneRotations(std::vector<Matrix>& rotations) const
{

}

}
 