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
    m_cumulativeRotation    = glm::identity<Quaternion>();
    m_localOrientation      = glm::identity<Quaternion>();

    m_poses.push_back(Pose{}); // add default pose;
    m_joints.push_back(Vector{0,0,0});
}

void Solver::OverrideRootPosition(const Vector& rootPosition)
{
    // TODO: update existing bone structure or throw an exception
    Vector delta     = rootPosition - m_joints.front();
    for (auto& joint : m_joints)
    {
        joint += rootPosition;
    }
}

void Solver::AddBone(real length, const Quaternion& orientation)
{
    auto& skeleton      = m_poses[m_defaultPose].bones;

    // add new bone to the chain and calculate absolute orientation of the axis
    m_localOrientation  = glm::normalize(m_localOrientation * orientation);
    Vector axis         = m_joints.back() + (m_localOrientation * Helpers::DefaultAxis() * length);

    skeleton.emplace_back(length, orientation);
    // form the new joint position
    m_joints.emplace_back(axis);
}

bool Solver::SetConstraint(size_t boneIndex, Constraints&& constraint)
{
    auto& chain = m_poses[m_defaultPose].bones;

    if (boneIndex < chain.size())
    {
        chain[boneIndex].SetConstraints(std::move(constraint));
        return true;
    }
    
    return false;
}

Pose&  Solver::GetBones(size_t poseIndex)
{
    return m_poses[poseIndex];
}

void Solver::CompleteChain()
{
    auto& chain = m_poses[m_defaultPose].bones;
    Quaternion base = glm::identity<Quaternion>();
    for (auto& bone : chain)
    {
        base = bone.GetRotation() * base;
        bone.SetGlobalOrientation(base);
    }
}

Vector Solver::GetTipPosition() const
{ 
    return m_joints.back(); 
}

void Solver::SetTargetPosition(const Vector& target)
{
    m_target = target;
}

void Solver::LookAt(const Vector& initialDirection, const Vector& target)
{
    // look at
    if (glm::length2(target) > EPSILON)
    {
        m_cumulativeRotation = Helpers::CalculateRotation(glm::normalize(initialDirection), glm::normalize(target)) * m_cumulativeRotation;
    }
}

void Solver::IterateFront()
{
    // front kinematics
    auto& chain = m_poses[m_defaultPose].bones;
    Quaternion rotation                 = glm::identity<Quaternion>();

    for (size_t i = 0; i < chain.size(); ++i)
    {
        // calculate cumuilative change of orientation of the current bone
        rotation                    = chain[i].GetRotation() * rotation;
        // calculate the global orientation of the bone
        Quaternion baseRotation     = rotation * chain[i].GetGlobalOrientation();
        // find the new position of the bone base joint
        Vector axis = (baseRotation * Helpers::DefaultAxis());
        m_joints[i + 1]             = m_joints[i] + (axis * chain[i].GetLength());
        chain[i].SetGlobalOrientation(baseRotation);
    }
}

void Solver::IterateBack()
{   
    // inverse kinematics: iterative
    auto& chain = m_poses[m_defaultPose].bones;

    if (chain.empty())
    {
        return;
    }

    // Assume distance to target is reachable
    Vector target           = m_target - m_joints.front();
    m_cumulativeRotation    = glm::identity<Quaternion>();

    Vector chainTip         = m_joints.back() - m_joints.front();

    for (size_t i = chain.size(); i > 1; --i)
    {
        const size_t jointIndex = i - 1;
        
        // rotate the root part of the chain according to the accumulated rotations
        Vector currentJoint = m_cumulativeRotation * (m_joints[jointIndex] - m_joints.front());
        // calculate simple joint consists of chain before and after the joint
        Vector tip = chainTip - currentJoint;
        if (glm::length2(tip) < EPSILON)
        {
            // if arm length is equal to 0, the step cannot provide any position change, skip it;
            continue;
        }
        chainTip = SolveBinaryJoint(chain[jointIndex], currentJoint, tip, target);
    }
    
    // final step, the chain might not reach the final direction, due to joint stiffness
    // do the final rotation of the root bone (if possible)
    LookAt(chainTip, target);

    // for the root joint all rotations are global
    chain.front().SetRotation(m_cumulativeRotation);
}

Vector Solver::SolveBinaryJoint(Bone& bone, const Vector& root, const Vector& tip, const Vector& target)
{
    // position local coordinate system to have root bone aligned with Y axis and with target forms XoY plane.
    // Make the working plane, the plane made by 2 vectors: initial arm and vector to target
    const Vector y          = glm::normalize(root);
    const Vector z          = Helpers::Normal(y, glm::normalize(target));
    const Vector x          = glm::normalize(glm::cross(z, y));

    Length lengthRoot(glm::length2(root));
    Length lengthTip(glm::length2(tip));
    // calculate angles required to reach the target with current binary joint
    auto angles             = CalculateAngles(lengthRoot, lengthTip, {glm::dot(target, x), glm::dot(target, y)});
    // calculate modifications for the chain root
    Quaternion rootRotation = glm::angleAxis(glm::pi<real>() / (real)2.0 - angles.first, z); 

    // rotate whole chain according to root rotation to calculate relative tip rotation angle.
    Vector newRoot          = rootRotation * y;
    Vector currentTip       = rootRotation * glm::normalize(tip);

    real tipFullAngle       = angles.first - angles.second;
    Vector newTip           = x * glm::cos(tipFullAngle) + y * glm::sin(tipFullAngle);

    // update the position of the chain tip
    m_cumulativeRotation    = glm::normalize(rootRotation * m_cumulativeRotation);

    // apply constraints
    auto& constraint        = bone.GetConstraints();
    auto tipRotationParams  = Helpers::CalculateParameters(currentTip, newTip);
    Quaternion tipRotation  = glm::angleAxis(tipRotationParams.angle * constraint.flexibility, tipRotationParams.axis);
    newTip                  = tipRotation * currentTip;
    
    auto& chain = m_poses[m_defaultPose].bones;
    bone.SetRotation(tipRotation); 

    return (newTip * lengthTip.l + newRoot * lengthRoot.l);
}

std::pair<real, real> Solver::CalculateAngles(const Length& root, const Length& tip, Vector2 chord) const
{
    // according to algorithm, x cannot be negative, but it is possible due to FP error,
    // assuming that algorithm is correct with faith in our harts enforce x to 0 and hope that it will not spoil the result
    chord.x                 = std::max(chord.x, 0.0);
    
    // 1st part of the rule of triangle x < y + z
    real chordLength        = glm::clamp(glm::length(chord), root.l - tip.l, root.l + tip.l);
    real lbsq               = chordLength * chordLength;
    // calculate local angles on the given coordinate system
    // TODO: check low values of chord.y
    real angleChord         = (chord.x > EPSILON) ? glm::atan(chord.y/chord.x) : glm::sign(chord.y) * glm::pi<real>() / 2.0;
 
    // according to the article, calculate position of bones on the coordinate system, 
    // https://www.learnaboutrobots.com/inverseKinematics.htm
    // Angle between x axis and new direction of the root
    // TODO: check clamp, maybe not needed?
    real angleRoot          = lbsq > EPSILON 
                            ? angleChord + glm::acos(glm::clamp((root.l2 - tip.l2 + lbsq) / (2 * root.l * chordLength), (real)-1., (real)1.)) 
                            : 0;
    // According the article angle between root and tip can be calculated this way
    real angleJoint         = glm::acos(glm::clamp((root.l2 + tip.l2 - lbsq) / (2 * root.l * tip.l), (real)-1., (real)1.));
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
        real angle         = glm::orientedAngle(test, Vector(testResult), testAxis );

        std::cout << "<-> rotation state: (" << test.x << "; " << test.y << "; " << test.z << ") -> (" << testResult.x << "; " << testResult.y << "; " << testResult.z << ")" << std::endl;
        std::cout << "<->               : (" << testAxis.x << "; " << testAxis.y << "; " << testAxis.z << ") angle: " << angle << std::endl;
    }
}

}
 