/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "constraint_solvers.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "glm/gtx/euler_angles.hpp"

#include <iostream>
#include <iomanip>
#include <functional>
#include <array>

namespace LightIK
{
    std::unique_ptr<ConstraintSolver> SolversFactory::BuildSolver(ConstraintModes solverMode, const Constraints& constraint)
    {
        using CreatorFunction = std::function<std::unique_ptr<ConstraintSolver>( const Constraints& )>;
        static std::array<CreatorFunction, (size_t)ConstraintModes::Count> functions = 
        {
            [](const Constraints& constraint){ return std::make_unique<PassThrough>(constraint);},
            [](const Constraints& constraint){ return std::make_unique<SwingTwistSolver>(constraint);},
            [](const Constraints& constraint){ return std::make_unique<ConstraintSolverXZY>(constraint);},
            [](const Constraints& constraint){ return std::make_unique<ConstraintSolverZXY>(constraint);},
            [](const Constraints& constraint){ return std::make_unique<ConstraintSolverYXZ>(constraint);},
            [](const Constraints& constraint){ return std::make_unique<ConstraintSolverYZX>(constraint);},
            [](const Constraints& constraint){ return std::make_unique<ConstraintSolverXYZ>(constraint);}
        };
        
        return functions[(size_t)solverMode](constraint);
    }

    // Constraint solvers
    // Swing and Twist solver
    SwingTwistSolver::SwingTwistSolver(const Constraints& constraint)
    {
        m_minTwist          = constraint.minAngles.y;
        m_maxTwist          = constraint.maxAngles.y;

        // Check if axises are not hard locked
        m_unlockedX         = (glm::abs(constraint.minAngles.x - constraint.maxAngles.x) > EPSILON);
        m_unlockedZ         = (glm::abs(constraint.minAngles.z - constraint.maxAngles.z) > EPSILON);

        // Do some calculations. half angles allow to process full scale rotations without facing issue related to sin symmetry to pi/2 
        real sMinX          = glm::sin((constraint.minAngles.x) * 0.5);
        real sMaxX          = glm::sin((constraint.maxAngles.x) * 0.5);

        real sMinZ          = glm::sin((constraint.minAngles.z) * 0.5);
        real sMaxZ          = glm::sin((constraint.maxAngles.z) * 0.5);

        // Calculate the center of the ellipse if not defined
        m_centerX           = (constraint.restAngles.x > constraint.minAngles.x && constraint.restAngles.x < constraint.maxAngles.x)
                                ? glm::sin((constraint.restAngles.x) * 0.5) : (sMinX + sMaxX) * 0.5;

        m_centerZ           = (constraint.restAngles.z > constraint.minAngles.z && constraint.restAngles.z < constraint.maxAngles.z)
                                ? glm::sin((constraint.restAngles.z) * 0.5) : (sMinZ + sMaxZ) * 0.5;

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

    Quaternion SwingTwistSolver::CalculateRotation(const Quaternion& rotation)
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

    // Calculate Tait-Bryan angles calculatation in non-standard seqence: XZY
    //
    EulerSolver::EulerSolver(const Constraints& constraint)
    {
        m_constraints = constraint; 
    }

    Quaternion EulerSolver::CalculateRotation(const Quaternion& rotation)
    {
        {
        Vector angles   = ToTaitBriant(rotation);
        angles          = glm::clamp(angles, m_constraints.minAngles, m_constraints.maxAngles);
        return FromTaitBriant(angles);
    }
    }

    Vector ConstraintSolverXZY::ToTaitBriant(const Quaternion& q)
    { 
        // TODO: need to prove that manual calculation is faster than standard glm
        // auto testMatrix = glm::mat4_cast(q);
        // Vector ref;
        // glm::extractEulerAngleXZY(testMatrix, ref.x, ref.z, ref.y);
        Vector result = {0, 0, glm::asin(glm::clamp((real)(2) * (q.w * q.z - q.x * q.y), (real)-1, (real)1))};

        // calculate X and Y angles
        Vector2 params = {
            (real)2 * (q.w * q.x + q.y * q.z),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z};
        if (!glm::all(glm::equal(params, Vector2(0,0), EPSILON)))
        {
            result.x = glm::atan2(params.x, params.y);
        }

        params.x = (real)2 * (q.w * q.y + q.x * q.z);
        params.y = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
        if (!glm::all(glm::equal(params, Vector2(0,0), EPSILON)))
        {
            result.y = glm::atan2(params.x, params.y);
        }

        return result;
    }

    // Calculate quaternion from Tait-Bryaint angles directly without applying heavy triple q multiplication
    Quaternion ConstraintSolverXZY::FromTaitBriant(const Vector& angles)
    {
        const Vector s = glm::sin(angles * (real)0.5);
        const Vector c = glm::cos(angles * (real)0.5);
        // calculate multiplication of 3 quaternions for each euler angle in sequence YXZ
        // Q = Qx * Qz * Qy
        return Quaternion{
            (c.x * c.y * c.z) + (s.x * s.y * s.z),
            (s.x * c.y * c.z) - (c.x * s.y * s.z),
            (c.x * s.y * c.z) - (s.x * c.y * s.z),
            (c.x * c.y * s.z) + (s.x * s.y * c.z)
        };
    }
    
    Vector ConstraintSolverZXY::ToTaitBriant(const Quaternion& q)
    { 
        Matrix4 m = glm::mat4_cast(q);
        Vector result {0,0,0};
        glm::extractEulerAngleZXY(m, result.z, result.x, result.y);
        
        return result;
    }

    // Calculate quaternion from Tait-Bryaint angles directly without applying heavy triple q multiplication
    Quaternion ConstraintSolverZXY::FromTaitBriant(const Vector& angles)
    {
        const Vector s = glm::sin(angles * (real)0.5);
        const Vector c = glm::cos(angles * (real)0.5);
        // calculate multiplication of 3 quaternions for each euler angle in sequence YXZ
        // Q = Qz * Qx * Qy
        return Quaternion{
            (c.x * c.y * c.z) - (s.x * s.y * s.z),
            (s.x * c.y * c.z) - (c.x * s.y * s.z),
            (c.x * s.y * c.z) + (s.x * c.y * s.z),
            (c.x * c.y * s.z) + (s.x * s.y * c.z)
        };
    }

    Vector ConstraintSolverYXZ::ToTaitBriant(const Quaternion& q)
    { 
        Vector result {0,0,0};
        glm::extractEulerAngleYXZ(glm::mat4_cast(q), result.y, result.x, result.z);
        
        return result;
    }

    // Calculate quaternion from Tait-Bryaint angles directly without applying heavy triple q multiplication
    Quaternion ConstraintSolverYXZ::FromTaitBriant(const Vector& angles)
    {
        const Vector s = glm::sin(angles * (real)0.5);
        const Vector c = glm::cos(angles * (real)0.5);
        // calculate multiplication of 3 quaternions for each euler angle in sequence YXZ
        // Q = Qy * Qx * Qz
        return Quaternion{
            (c.x * c.y * c.z) + (s.x * s.y * s.z),
            (s.x * c.y * c.z) + (c.x * s.y * s.z),
            (c.x * s.y * c.z) - (s.x * c.y * s.z),
            (c.x * c.y * s.z) - (s.x * s.y * c.z)
        };
    }

    Vector ConstraintSolverYZX::ToTaitBriant(const Quaternion& q)
    { 
        Vector result {0,0,0};
        glm::extractEulerAngleYZX( glm::mat4_cast(q), result.y, result.z, result.x);
        
        return result;
    }

    // Calculate quaternion from Tait-Bryaint angles directly without applying heavy triple q multiplication
    Quaternion ConstraintSolverYZX::FromTaitBriant(const Vector& angles)
    {
        const Vector s = glm::sin(angles * (real)0.5);
        const Vector c = glm::cos(angles * (real)0.5);
        // calculate multiplication of 3 quaternions for each euler angle in sequence YXZ
        // Q = Qy * Qz * Qx
        return Quaternion{
            (c.x * c.y * c.z) - (s.x * s.y * s.z),
            (s.x * c.y * c.z) - (c.x * s.y * s.z),
            (c.x * s.y * c.z) + (s.x * c.y * s.z),
            (c.x * c.y * s.z) - (s.x * s.y * c.z)
        };
    }

    
    Vector ConstraintSolverXYZ::ToTaitBriant(const Quaternion& q)
    { 
        //Vector result {0,0,0};
        //;
        
        return glm::eulerAngles(q);
    }

    // Calculate quaternion from Tait-Bryaint angles directly without applying heavy triple q multiplication
    Quaternion ConstraintSolverXYZ::FromTaitBriant(const Vector& angles)
    {
        return Quaternion{angles};
    }
}
