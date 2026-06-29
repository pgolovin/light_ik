/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "bone.h"
#include "solver.h"
#include "helpers.h"
#include "constraint_solvers.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{

Bone::Bone()
    : m_solver(SolversFactory::BuildSolver(ConstraintModes::PassThrough, Constraints()))
{
    m_initialRotation   = glm::identity<Quaternion>();
    m_rotation          = glm::identity<Quaternion>();
    m_globalOrientation = glm::identity<Quaternion>();
}

Bone::Bone(real length, const Quaternion& orientation)
    : m_rotation(orientation)
    , m_initialRotation(orientation)
    , m_length(length, false)
    , m_solver(SolversFactory::BuildSolver(ConstraintModes::PassThrough, Constraints()))
{
    m_globalOrientation = glm::identity<Quaternion>();

}

void Bone::ForceRotation(const Quaternion& orientation)
{
    // relative rotation according to the parent orientation
    m_rotation          = orientation;
}

void Bone::SetGlobalOrientation(const Quaternion& orientation)
{
    // skeleton global orientation
    m_globalOrientation = orientation;
}

void Bone::SetConstraints(Constraints && newConstraints)
{
    m_constraints       = std::move(newConstraints);
    m_solver            = SolversFactory::BuildSolver(m_constraints.mode, newConstraints);
}

Quaternion Bone::ApplyRotation(const Quaternion& rotation)
{
    m_rotation = m_solver->CalculateRotation(rotation);
    return m_rotation;
}

void Bone::Reset()
{ 
    m_rotation          = m_initialRotation;
    m_globalOrientation = glm::identity<Quaternion>();
}

}
