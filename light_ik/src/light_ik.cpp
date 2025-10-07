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

Vector LightIK::GetTargetPosition()const
{
    return m_solver->GetTargetPosition();
}

Vector LightIK::GetTipPosition()const
{
    return m_solver->GetTipPosition();
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
    rotations.clear();
    rotations.resize(m_solver->GetChainSize());

}

 void LightIK::GetRotationParameters(std::vector<RotationParameters>& rotations) const
 {
    rotations.clear();
    rotations.resize(m_solver->GetChainSize());

    for (size_t b = 0; b < m_solver->GetChainSize(); ++b)
    {
        //rotations[b] = m_solver->GetBone(b).GetChainRotation(m_solver->GetBone(b).GetAxis());
    }
 }

 void LightIK::GetRelativeRotations(std::vector<RotationParameters>& rotations, Vector initialDirection) const
 {
    rotations.clear();
    rotations.reserve(m_solver->GetChainSize());

    for (size_t b = 0; b < m_solver->GetChainSize(); ++b)
    {
        Vector currentBone = m_solver->GetBone(b).GetAxis();
        Vector rotationAxis = Helpers::Normal(initialDirection, currentBone);
        float angle = glm::orientedAngle(initialDirection, currentBone, rotationAxis);
        rotations.emplace_back(RotationParameters{rotationAxis, angle});
        initialDirection = currentBone;
    }
 }

}
 