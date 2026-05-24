/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "bone.h"
#include "solver.h"
#include "helpers.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{

const std::unique_ptr<ConstraintSolver> constraintSolvers[(size_t)ConstraintModes::Count] = 
{
    std::make_unique<ConstraintSolverXZY>(), 
    std::make_unique<ConstraintSolverZXY>(),
    std::make_unique<ConstraintSolverYXZ>(),
};

Bone::Bone()
    : m_solver(std::ref(*constraintSolvers[0]))
{
    m_initialRotation           = glm::identity<Quaternion>();
    m_rotation                  = glm::identity<Quaternion>();
    m_globalOrientation         = glm::identity<Quaternion>();
}

Bone::Bone(real length, const Quaternion& orientation)
    : m_rotation(orientation)
    , m_initialRotation(orientation)
    , m_length(length, false)
    , m_solver(std::ref(*constraintSolvers[0]))
{
    m_globalOrientation         = glm::identity<Quaternion>();
}

void Bone::SetRotation(const Quaternion& orientation)
{
    // relative rotation according to the parent orientation
    m_rotation                  = orientation;
}

void Bone::SetGlobalOrientation(const Quaternion& orientation)
{
    // skeleton global orientation
    m_globalOrientation         = orientation;
}

void Bone::SetConstraints(Constraints && newConstraints)
{
    m_constraints               = std::move(newConstraints);
    m_solver                    = std::ref(*constraintSolvers[(size_t)m_constraints.mode]);
}

Quaternion Bone::ApplyConstraint(const Quaternion& inverseParent, const Quaternion& rotation) const
{
    {
        Vector angles           = m_solver.get().ToTaitBriant(inverseParent * rotation);
        angles                  = glm::clamp(angles, m_constraints.minAngles, m_constraints.maxAngles);
        return m_solver.get().FromTaitBriant(angles);
    }
}

void Bone::Reset()
{ 
    m_rotation                  = m_initialRotation;
    m_globalOrientation         = glm::identity<Quaternion>();
}

}
