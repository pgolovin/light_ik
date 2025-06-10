/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "solver.h"
#include "helpers.h"
#include "glm/gtx/norm.inl"
#include "glm/gtx/vector_angle.inl"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{

Solver::Solver()
{
    m_poses.push_back(Pose{}); // add default pose;
}

void Solver::OverrideRootPosition(const Vector& rootPosition)
{
    m_root = rootPosition;
}

void Solver::AddBone(const Vector& boneEnd)
{
    m_lastBone = &m_poses[m_defaultPose].bones.emplace_back(m_chainTip, boneEnd, m_lastBone->GetAxis());

    // form the bone origin
    size_t index = m_poses[m_defaultPose].bones.size();
    m_chainTip = boneEnd;
}

const Bone& Solver::GetBone(size_t index) const
{
    return m_poses[m_defaultPose].bones[index];
}

void Solver::SetTargetPosition(const Vector& target)
{
    m_target = target;
}

void Solver::IterateBack()
{
    // inverse kinematics: iterative
    auto& chain             = m_poses[m_defaultPose].bones;

    // Assume distance to target is reachable
    Vector target           = m_target - m_root;
    m_accumulatedRotation   = glm::identity<Matrix>();

    m_testRoot = chain.size() ? chain.front().GetAxis() : Vector{0.f, 0.f, 0.f};

    for (size_t index = chain.size(); index > 1; --index)
    {
        SolveFlatJoint(m_poses[m_defaultPose], target, index - 1);
    }

    if (1 == chain.size())
    {
        LookAt(chain.front());
    }

    if constexpr (EnableDebugLogging)
    {
        std::cout << "<- back ct: " << m_chainTip.x << "; " << m_chainTip.y << "; " << m_chainTip.z << std::endl;
    }
}

void Solver::LookAt(Bone& lookAtBone)
{
    // look at
    Vector newDirection = m_target - lookAtBone.GetRoot();
    if (glm::length2(newDirection) > DELTA)
    {
        newDirection = glm::normalize(newDirection);
        float angle = glm::angle(lookAtBone.GetAxis(), newDirection);
        if (angle > DELTA)
        {
            Vector axis = glm::cross(lookAtBone.GetAxis(), newDirection);

            if (glm::length2(axis) < DELTA)
            {
                axis = lookAtBone.GetFront();
            }
            lookAtBone.Rotate(axis, angle);
            m_chainTip = m_root + lookAtBone.GetAxis() * lookAtBone.GetLength();
        }
    }
}

void Solver::IterateFront()
{
    //front kinematics
    auto& chain = m_poses[m_defaultPose].bones;

    if (chain.empty())
    {
        return;
    }
    
    m_chainTip = m_root;
    Matrix rotation = m_accumulatedRotation;
    for (auto& bone : chain)
    {
        bone.Rotate(rotation);
        rotation = bone.GetRotation();
        m_chainTip += bone.GetAxis() * bone.GetLength();

        if constexpr (EnableDebugLogging)
        {
            std::cout << bone.GetAxis().x << "; " << bone.GetAxis().y << "; " << bone.GetAxis().z << std::endl;
        }
    }

    if constexpr (EnableDebugLogging)
    {
        std::cout << "-> front ct: " << m_chainTip.x << "; " << m_chainTip.y << "; " << m_chainTip.z << std::endl;
    }
}

void Solver::SolveFlatJoint(Pose& chain, const Vector& target, size_t jointIndex)
{
    // according to the article, calculate position of bones on the coordinate system, 
    // https://www.learnaboutrobots.com/inverseKinematics.htm
    // position local coordinate system to have root bone aligned with Y axis and with target forms XoY plane.
    
    // apply cumulated root rotations for current joint, to calculate its real position
    Vector joint            = m_accumulatedRotation * Vector4(chain.bones[jointIndex].GetRoot(), 1);
    Vector armRoot          = joint - m_root;
    Vector armTip           = m_chainTip - joint;

    float l2                = glm::length(armTip);
    if (l2 < DELTA)
    {
        // if arm length is equal to 0, the step cannot provide any position change, skip it;
        return;
    }
    // normalize all vector to simplify rotation calculatuions
    armTip                  = glm::normalize(armTip);

    float l1                = glm::length(armRoot);

    // Make the working plane, the plane made by 2 vectors: initial arm and vector to target
    const Vector y          = glm::normalize(armRoot);
    const Vector z          = Helpers::Normal(y, target);
    const Vector x          = glm::normalize(glm::cross(z, y));

    // calculate arm lengths and distance from root to target
    Vector2 b               = Vector2{glm::dot(target, x), glm::dot(target, y)};
    assert(b.x >= 0);
    // 1st part of the rule of triangle x < y + z
    float lb                = glm::clamp(glm::length(b), l1 - l2, l1 + l2);

    // calculate local angles on the given coordinate system
    float angleChord        = 0;
    if (glm::abs(b.y) > DELTA)
    {
        angleChord = (b.x > DELTA) ? glm::atan(b.y/b.x) : glm::sign(b.y) * glm::pi<float>() / 2.f;
    }  

    float l1sq = l1*l1;
    float l2sq = l2*l2;
    float lbsq = lb*lb;

    float angleRoot         = lb > DELTA ? angleChord + glm::acos((l1sq - l2sq + lbsq) / (2 * l1 * lb)) : 0;
    float angleJoint        = angleRoot + glm::acos((l1sq + l2sq - lbsq) / (2 * l1 * l2)) - glm::pi<float>(); // TODO: check if PI substraction is needed
    
    // calculate modifications for the chain root
    Matrix currentRotation  = glm::rotate(glm::identity<Matrix>(), glm::pi<float>() / 2.f - angleRoot, z);  

    // rotate whole chain according to root rotation to calculate relative tip rotation angle.
    Vector currentTip       = Vector(currentRotation * Vector4(armTip, 1));
    armRoot                 = Vector(currentRotation * Vector4(y, 1));

    // calculate modifications for the top part of the chain
    armTip                  = x * glm::cos(angleJoint) + y * glm::sin(angleJoint);
    Vector axisTipRotation  = Helpers::Normal(armTip, currentTip);

    float newAngle          = glm::orientedAngle(glm::normalize(currentTip), armTip, axisTipRotation);
    Matrix tipRotation      = glm::rotate(glm::identity<Matrix>(), newAngle, axisTipRotation); 
    chain.bones[jointIndex].Rotate(tipRotation);

    if constexpr (EnableDebugLogging)
    {
        Vector checkVectorTip   = tipRotation * Vector4(currentTip, 1);
        float angle = glm::angle(glm::normalize(checkVectorTip), armTip);
        if (std::isnan(angle))
        {
            assert(false);
        }

        std::cout << "difference angle: " << angle << std::endl;
        assert(glm::length2(checkVectorTip - armTip) < DELTA);
    }

    // update the position of the chain tip
    m_chainTip              = armTip * l2 + armRoot * l1;
    m_accumulatedRotation   = m_accumulatedRotation * currentRotation;
}

}
 