/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "solver.h"
#include "helpers.h"
#include "glm/gtx/vector_angle.inl"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/rotate_vector.hpp"
//#include "glm/gtx/quaternion.hpp"

#include <iostream>

namespace LightIK
{

Solver::Solver()
{
    m_cumulativeRotation = glm::identity<Quaternion>();
    m_poses.push_back(Pose{}); // add default pose;
}

void Solver::OverrideRootPosition(const Vector& rootPosition)
{
    // TODO: update existing bone structure or throw an exception
    m_root = rootPosition;
    m_chainTip = m_root;
}

void Solver::AddBone(const Vector& boneEnd)
{
    m_lastBone = &m_poses[m_defaultPose].bones.emplace_back(m_chainTip, boneEnd, m_lastBone->GetAxis());

    // form the bone origin
    size_t index = m_poses[m_defaultPose].bones.size();
    m_chainTip = boneEnd;
}

Bone& Solver::GetBone(size_t index)
{
    return m_poses[m_defaultPose].bones[index];
}
Vector Solver::GetTipPosition() const
{ 
    return m_chainTip; 
}

void Solver::SetTargetPosition(const Vector& target)
{
    m_target = target;
}

void Solver::LookAt(Bone& lookAtBone)
{
    // look at
    Vector newDirection = m_target - lookAtBone.GetRoot();
    if (glm::length2(newDirection) > DELTA)
    {
        newDirection = glm::normalize(newDirection);
        real angle = glm::angle(lookAtBone.GetAxis(), newDirection);
        if (angle > DELTA)
        {
            Vector axis = Helpers::Normal(lookAtBone.GetAxis(), newDirection);
            m_cumulativeRotation = glm::angleAxis(angle, axis);
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

    Quaternion chainRotation = glm::identity<Quaternion>();
    m_chainTip = m_root;
    for (auto& bone : chain)
    {
        chainRotation = bone.Rotate(chainRotation);
        m_chainTip += bone.GetAxis() * bone.GetLength();
    }

    std::cout << ">>F>> tip: " << m_chainTip.x << ", "<< m_chainTip.y << ", "<< m_chainTip.z << std::endl;
}

void Solver::IterateBack()
{   
    // inverse kinematics: iterative
    auto& chain = m_poses[m_defaultPose].bones;

    if (chain.empty())
    {
        return;
    }

    if (1 == chain.size())
    {
        LookAt(chain.front());
        return;
    }

    // Assume distance to target is reachable
    Vector target           = m_target - m_root;
    m_cumulativeRotation    = glm::identity<Quaternion>();

    for (size_t i = chain.size(); i > 1; --i)
    {
        const size_t jointIndex = i - 1;

        const Vector localJoint = chain[jointIndex].GetRoot() - m_root;
        
        // rotate the root part of the chain according to the accumulated rotations
        Vector currentJoint = m_cumulativeRotation * localJoint;
        // calculate simple joint consists of chain before and after the joint
        auto joint = FormBinaryJoint(currentJoint);
        if (joint.first.length < DELTA)
        {
            // if arm length is equal to 0, the step cannot provide any position change, skip it;
            continue;
        }
        auto qRotation = SolveBinaryJoint(joint.first, joint.second, target);
        chain[jointIndex].SetRotation(qRotation);
    }
    chain.front().SetRotation(m_cumulativeRotation);
}

std::pair<Solver::CompaundVector, Solver::CompaundVector> Solver::FormBinaryJoint(const Vector& joint) const
{
    CompaundVector armRoot{joint, 0};
    armRoot.length2         = glm::length2(armRoot.direction);
    armRoot.length          = glm::sqrt(armRoot.length2);
    armRoot.direction       = glm::normalize(armRoot.direction);

    CompaundVector armTip{m_chainTip - m_root - joint, 0};
    armTip.length2          = glm::length2(armTip.direction);
    armTip.length           = glm::sqrt(armTip.length2);
    armTip.direction        = glm::normalize(armTip.direction);

    return {armRoot, armTip};
}

Quaternion Solver::SolveBinaryJoint(Solver::CompaundVector& root, Solver::CompaundVector& tip, const Vector& target)
{
    // position local coordinate system to have root bone aligned with Y axis and with target forms XoY plane.
    // Make the working plane, the plane made by 2 vectors: initial arm and vector to target
    const Vector y          = root.direction;
    const Vector z          = Helpers::Normal(y, target);
    const Vector x          = glm::normalize(glm::cross(z, y));

    // calculate angles required to reach the target with current binary joint
    auto angles             = CalculateAngles(root, tip, {glm::dot(target, x), glm::dot(target, y)});
    // calculate modifications for the chain root
    Quaternion rootRotation = glm::angleAxis(glm::pi<real>() / 2.f - angles.first, z); 

    // rotate whole chain according to root rotation to calculate relative tip rotation angle.
    root.direction          = rootRotation * y;
    Vector currentTip       = rootRotation * tip.direction;
    real tipFullAngle       = angles.first - angles.second;
    tip.direction           = x * glm::cos(tipFullAngle) + y * glm::sin(tipFullAngle);

    // update the position of the chain tip
    m_chainTip              = m_root + tip.direction * tip.length + root.direction * root.length;
    m_cumulativeRotation    = glm::normalize(rootRotation * m_cumulativeRotation);

    //Calculate an axis and an angle between old and new position of the tip
    RotationParameters deltaRotation;
    deltaRotation.axis      = Helpers::Normal(currentTip, tip.direction);
    deltaRotation.angle     = glm::orientedAngle(currentTip, tip.direction, deltaRotation.axis);
    return glm::angleAxis(deltaRotation.angle, deltaRotation.axis);
}

std::pair<real, real> Solver::CalculateAngles(const Solver::CompaundVector& root, const Solver::CompaundVector& tip, Vector2 chord) const
{
    if(chord.x < -DELTA)
    {
        std::cout << ">>>>>>> HEH?" << std::endl << ">>>>>>> root = " << glm::length(root.direction) << " tip = " << glm::length(tip.direction) << std::endl;
    }
    if (chord.x > -DELTA && chord.x < 0)
    {
        chord.x             = 0;
    }
    
    // 1st part of the rule of triangle x < y + z
    real chordLength        = glm::clamp(glm::length(chord), root.length - tip.length, root.length + tip.length);
    real lbsq               = chordLength * chordLength;
    // calculate local angles on the given coordinate system
    // TODO: check low values of chord.y
    real angleChord         = (chord.x > DELTA) ? glm::atan(chord.y/chord.x) : glm::sign(chord.y) * glm::pi<float>() / 2.f;
 
    // according to the article, calculate position of bones on the coordinate system, 
    // https://www.learnaboutrobots.com/inverseKinematics.htm
    // Angle between x axis and new direction of the root
    // TODO: check clamp, maybe not needed?
    real angleRoot          = lbsq > DELTA 
                            ? angleChord + glm::acos(glm::clamp((root.length2 - tip.length2 + lbsq) / (2 * root.length * chordLength), (real)-1., (real)1.)) 
                            : 0;
    // According the article angle between root and tip can be calculated this way
    real angleJoint         = glm::acos(glm::clamp((root.length2 + tip.length2 - lbsq) / (2 * root.length * tip.length), (real)-1., (real)1.));
    // Modify the angle, to make it the angle between previous bone axis and actual direction on the arm tip.
    angleJoint              = glm::pi<real>() - angleJoint;

    return {angleRoot, angleJoint};    
}

void Solver::ValidateRotationMatrix(const Matrix& rotation, const Vector& testVector) const
{
    if constexpr (EnableDebugLogging)
    {
        Vector test         = glm::normalize(testVector);
        Vector testResult   = rotation * Vector4(test, 1);
        Vector testAxis     = glm::normalize(glm::cross(test, testResult));
        float angle         = glm::orientedAngle(test, Vector(testResult), testAxis );

        std::cout << "<-> rotation state: (" << test.x << "; " << test.y << "; " << test.z << ") -> (" << testResult.x << "; " << testResult.y << "; " << testResult.z << ")" << std::endl;
        std::cout << "<->               : (" << testAxis.x << "; " << testAxis.y << "; " << testAxis.z << ") angle: " << angle << std::endl;
    }
}

}
 