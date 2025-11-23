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

float LightIK::GetBoneLength(size_t index) const
{
    return m_solver->GetBone(index).GetLength();
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

void LightIK::GetBoneRotations(std::vector<Matrix>& rotations) const
{
    rotations.clear();
    rotations.resize(m_solver->GetChainSize());

}

std::vector<RotationParameters> LightIK::GetRotationParameters(Vector initialDirection) const
{
    std::vector<RotationParameters> rotations;
    // rotations.reserve(m_solver->GetChainSize());

    // for (size_t b = 0; b < m_solver->GetChainSize(); ++b)
    // {
    //     auto rotation = rotations.emplace_back(Helpers::CalculateParameters(initialDirection, m_solver->GetBone(b).GetAxis()));
    //     initialDirection = m_solver->GetBone(b).GetAxis();
    // }
    return rotations;
}

std::vector<Quaternion> LightIK::GetRelativeRotations(Vector initialDirection) const
{
    std::vector<Quaternion> rotations(m_solver->GetChainSize());
    // CoordinateSystem parent{glm::identity<CoordinateSystem>()};
    // const Bone& bone    = m_solver->GetBone(0);

    // CoordinateSystem local = bone.GetLocalCoordinateSystem();
    // Matrix rotation = Helpers::CalculateTransferMatrix(parent, local);

    // rotations[0] = glm::normalize(glm::quat_cast(rotation));
    return rotations;
}

std::vector<Matrix> LightIK::GetRelativeRotationMatrices(Vector initialDirection) const
{
    std::vector<Matrix> rotations(m_solver->GetChainSize());
    // CoordinateSystem parent{glm::identity<CoordinateSystem>()};
    // for (size_t b = 0; b < m_solver->GetChainSize(); ++b)
    // {
    //     const Bone& bone        = m_solver->GetBone(b);
    //     CoordinateSystem local  = bone.GetLocalCoordinateSystem();
    //     rotations[b] = Helpers::CalculateTransferMatrix(parent, local);
    //     parent = local;
    // }

    return rotations;
}

std::vector<RotationParameters> LightIK::GetRelativeRotationParameters(Vector initialDirection) const
{
    std::vector<RotationParameters> rotations(m_solver->GetChainSize());
    // CoordinateSystem local{glm::identity<CoordinateSystem>()};
    // Quaternion rotation = glm::identity<Quaternion>();
    // for (size_t b = 0; b < m_solver->GetChainSize(); ++b)
    // {
    //     const auto& bone = m_solver->GetBone(b);
    //     auto params      = Helpers::CalculateParameters(local[(size_t)Axis::y], bone.GetAxis());
    //     params.axis      = Helpers::ToLocal(local, params.axis);
    //     rotations[b]     = params;
    //     local            = bone.GetLocalCoordinateSystem();
    // }
    return rotations;
}

}
 