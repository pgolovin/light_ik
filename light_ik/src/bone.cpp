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
    std::make_unique<ConstraintSolverYZX>(),
};

Bone::Bone()
    : m_solver(std::ref(*constraintSolvers[0]))
{
    m_initialRotation   = glm::identity<Quaternion>();
    m_rotation          = glm::identity<Quaternion>();
    m_globalOrientation = glm::identity<Quaternion>();
}

Bone::Bone(real length, const Quaternion& orientation)
    : m_rotation(orientation)
    , m_initialRotation(orientation)
    , m_length(length, false)
    , m_solver(std::ref(*constraintSolvers[0]))
{
    m_globalOrientation = glm::identity<Quaternion>();
    m_minTwist          = -glm::pi<real>();
    m_maxTwist          = glm::pi<real>();
}

void Bone::SetRotation(const Quaternion& orientation)
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
    m_solver            = std::ref(*constraintSolvers[(size_t)m_constraints.mode]);

    m_minXLimit         = 0;
    m_maxXLimit         = 0;

    m_minZLimit         = 0;
    m_maxZLimit         = 0;

    m_minTwist          = newConstraints.minAngles.y;
    m_maxTwist          = newConstraints.maxAngles.y;

    // Check if axises are not hard locked
    m_unlockedX         = (glm::abs(newConstraints.minAngles.x - newConstraints.maxAngles.x) > EPSILON);
    m_unlockedZ         = (glm::abs(newConstraints.minAngles.z - newConstraints.maxAngles.z) > EPSILON);

    // Do some calculations. half angles allow to process full scale rotations without facing issue related to sin symmetry to pi/2 
    real sMinX          = glm::sin((newConstraints.minAngles.x) * 0.5);
    real sMaxX          = glm::sin((newConstraints.maxAngles.x) * 0.5);

    real sMinZ          = glm::sin((newConstraints.minAngles.z) * 0.5);
    real sMaxZ          = glm::sin((newConstraints.maxAngles.z) * 0.5);

    // Calculate the center of the ellipse if not defined
    m_centerX           = (newConstraints.restAngles.x > newConstraints.minAngles.x && newConstraints.restAngles.x < newConstraints.maxAngles.x)
                            ? glm::sin((newConstraints.restAngles.x) * 0.5) : (sMinX + sMaxX) * 0.5;

    m_centerZ           = (newConstraints.restAngles.z > newConstraints.minAngles.z && newConstraints.restAngles.z < newConstraints.maxAngles.z)
                            ? glm::sin((newConstraints.restAngles.z) * 0.5) : (sMinZ + sMaxZ) * 0.5;

    // Calculate the half axises of the fully unsymmetric ellipse
    if (m_unlockedX)
    {
        m_minXLimit     = 1. / glm::pow(sMinX - m_centerX, 2.);
        m_maxXLimit     = 1. / glm::pow(sMaxX - m_centerX, 2.);
    }

    if (m_unlockedZ)
    {
        m_minZLimit     = 1. / glm::pow(sMinZ - m_centerZ, 2.);
        m_maxZLimit     = 1. / glm::pow(sMaxZ - m_centerZ, 2.);
    }
}

Quaternion Bone::ApplyConstraint(const Quaternion& inverseParent, const Quaternion& rotation) const
{
    {
        Vector angles   = m_solver.get().ToTaitBriant(inverseParent * rotation);
        angles          = glm::clamp(angles, m_constraints.minAngles, m_constraints.maxAngles);
        return m_solver.get().FromTaitBriant(angles);
    }
}

Quaternion Bone::CalculateConstraintRotation(const Quaternion& rotation) const
{
    // Calculate swing and twist quaternions
    Quaternion twist    = glm::normalize(Quaternion{rotation.w, 0, rotation.y, 0});
    Quaternion swing    = rotation * glm::conjugate(twist);

    // Calculate current vector coordinates according to current center, moving to local coordinates
    real sx             = m_unlockedX ? swing.x - m_centerX : 0.;
    real sz             = m_unlockedZ ? swing.z - m_centerZ : 0.;

    // based on sign of the coordinates choose the right square of ellipse
    real halfAxisX      = sx < 0 ? m_minXLimit : m_maxXLimit;
    real halfAxisZ      = sz < 0 ? m_minZLimit : m_maxZLimit;

    // Calculate elliptic value of the current square
    real ellipticValue  = (sx * sx) * halfAxisX + (sz * sz) * halfAxisZ;
    if (ellipticValue > 1)
    {
        // if the angle is outside of the ellipse, scale it back to the ellipse edge
        real scale      = glm::inversesqrt(ellipticValue);
        sx              *= scale;
        sz              *= scale;
    }
    // return angles back to the global coordinates
    sx                  += m_centerX;
    sz                  += m_centerZ;

    // assemble the swing quaternion back
    real w              = 1.0 - (sx * sx) - (sz * sz);
    swing               = Quaternion{(w > 0 ? sqrt(w) : 0.), sx, 0.0, sz};

    // If swing.w is inversed comparing to original rotation, turn it back, to avoid large rotations
    if (swing.w * rotation.w < 0)
    {
        swing           = -swing;
    }
    
    // process twist
    real halfTwist = glm::atan(twist.y, twist.w);
    halfTwist = glm::clamp(halfTwist * 2., m_minTwist, m_maxTwist)/2.;

    twist.w = glm::cos(halfTwist);
    twist.y = glm::sin(halfTwist);

    if(!(glm::abs(twist.x) < EPSILON && glm::abs(twist.z) < EPSILON))
    {
        assert(false);
    }

    // assemble the full rotation
    return glm::normalize(swing * twist);
}

void Bone::Reset()
{ 
    m_rotation          = m_initialRotation;
    m_globalOrientation = glm::identity<Quaternion>();
}

}
