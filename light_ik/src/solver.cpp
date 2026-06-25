/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "solver.h"
#include "helpers.h"
#include "skeleton.h"
#include "glm/gtx/vector_angle.inl"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/rotate_vector.hpp"
//#include "glm/gtx/quaternion.hpp"

#include <iostream>

namespace LightIK
{

Solver::Solver(BoneSubchain&& chain, const Bone& parentBone, size_t pivotIndexInChain, Target& target)
    : m_chain(std::move(chain))
    , m_target(target)
    , m_parentBone(parentBone)
    , m_pivotBoneIndex(pivotIndexInChain)
{
    assert(m_chain.size());
    
    // assign owner for each bone in the chain
    for (auto& bone : m_chain)
    {
        bone.get().SetOwner(this);
    }
}

Vector Solver::GetRootPosition() const
{
    return m_chain[0].get().GetPosition();
}

const BoneSubchain& Solver::GetChain() const
{
    return m_chain;
}

void Solver::SetTipPosition(Vector& position) 
{
    m_tipPosition = position;
}

Vector Solver::GetTipPosition() const
{ 
    return m_tipPosition; 
}

void Solver::LookAt(ChainData& chainData, const Vector& target)
{
    // look at
    if (glm::length2(target) > EPSILON && glm::length2(chainData.tip) > EPSILON)
    {
        chainData.cumulativeRotation = 
            Helpers::CalculateRotation(glm::normalize(chainData.tip), glm::normalize(target)) * chainData.cumulativeRotation;
    }
}

void Solver::Execute()
{   
    // inverse kinematics: iterative
    if (m_chain.empty())
    {
        return;
    }
    
    // Check if chain consists of two subchains, i.e. has a pivot joint. 
    // if a pivot joint is available, the first subchain performs rotations from the pivot to the tip
    const Bone& parent  = m_pivotBoneIndex ? m_chain[m_pivotBoneIndex - 1] : m_parentBone;
    Bone& pivotBone     = m_chain.at(m_pivotBoneIndex);
    ChainData chainData = SolveSubchain(parent, pivotBone, m_chain.size() - 1, m_pivotBoneIndex);
    // If pivot is available, then the fist subchain will be calculated uisng pre-pivot joints, and the whole chain tip
    if (m_pivotBoneIndex)
    {
        Quaternion baseCumulativeRotation   = chainData.cumulativeRotation * pivotBone.GetRotation();
        Vector target                       = m_target.GetPosition() - pivotBone.GetPosition();
        LookAt(chainData, target);
        // finalize rotation of the pivot bone 
        //Quaternion finalAngle = pivotBone.ApplyConstraint(glm::identity<Quaternion>(), chainData.cumulativeRotation * pivotBone.GetRotation());
        Quaternion finalAngle = pivotBone.CalculateConstraintRotation(chainData.cumulativeRotation * pivotBone.GetRotation());
        pivotBone.SetRotation(finalAngle);

        // calculate new tip position based on LookAt orientation of the pivot bone
        Quaternion tipRotation = finalAngle * glm::inverse(baseCumulativeRotation);
        
        m_tipPosition   = (tipRotation * chainData.tip) + pivotBone.GetPosition();
        chainData       = SolveSubchain(m_parentBone, m_chain.front(), m_pivotBoneIndex - 1, 0);
    }

    Bone& rootBone      = m_chain.front();
    // Final step, the chain might not reach the final di rection, due to joint stiffness
    // do the final rotation of the root bone (if possible)
    Vector target       = m_target.GetPosition() - rootBone.GetPosition();
    LookAt(chainData, target);

    // Applying final constraints for the root bone
    auto parentOrientationInv  = glm::inverse(m_parentBone.GetGlobalOrientation());
    // Quaternion finalAngle = rootBone.ApplyConstraint(glm::identity<Quaternion>(), chainData.cumulativeRotation * rootBone.GetRotation());
    
    Quaternion finalAngle = rootBone.CalculateConstraintRotation(chainData.cumulativeRotation * rootBone.GetRotation());
    rootBone.SetRotation(finalAngle); 
}

Solver::ChainData Solver::SolveSubchain(const Bone& parentBone, Bone& rootBone, size_t tail, size_t base)
{
    // Assume distance to target is reachable
    Vector target           = m_target.GetPosition() - rootBone.GetPosition();

    // Construct base parameters of the current sub chain
    ChainData chainData { m_tipPosition - rootBone.GetPosition(), glm::identity<Quaternion>(), rootBone.GetRotation()};    
    
    // Calculate relative rotation of the current bone according to the orienation of its parent bone
    for (size_t i = tail; i > base; --i)
    {
        // rotate the root part of the chain according to the accumulated rotations
        Vector currentJoint = chainData.cumulativeRotation * (m_chain[i].get().GetPosition() -  rootBone.GetPosition());
        // calculate simple joint consists of chain before and after the joint
        Vector tip = chainData.tip - currentJoint;
        if (glm::length2(tip) < EPSILON)
        {
            // if arm length is equal to 0, the step cannot provide any position change, skip it;
            continue;
        }
        SolveBinaryJoint(chainData, rootBone, m_chain[i], m_chain[i - 1], currentJoint, tip, target);
    }

    return chainData;
}

bool Solver::TargetReached() const
{
    return glm::length2(m_tipPosition - m_target.GetPosition()) < EPSILON;
}

void Solver::SolveBinaryJoint(ChainData& chainData, Bone& baseBone, Bone& bone, const Bone& parent, const Vector& root, const Vector& tip, const Vector& target)
{
    // Position local coordinate system to have root bone aligned with Y axis and with target forms XoY plane.
    // Make the working plane, the plane made by 2 vectors: initial arm and vector to target
    const Vector y                  = glm::normalize(root);
    const Vector z                  = Helpers::Normal(y, glm::length2(target) > EPSILON ? glm::normalize(target) : Helpers::DefaultAxis());
    const Vector x                  = glm::normalize(glm::cross(z, y));

    auto& constraint                = bone.GetConstraints();

    // Calculate parameters of current joint
    Length lengthRoot(glm::length2(root));
    Length lengthTip(glm::length2(tip));
    // Calculate angles required to reach the target with current binary joint
    auto rawAngles                  = CalculateAngles(lengthRoot, lengthTip, {glm::dot(target, x), glm::dot(target, y)});
    
    // Calculate the set of base angles
    Quaternion rootRotation         = CalculateRootRotation(rawAngles.chord + rawAngles.root, chainData, z, baseBone);

    // Check if rotation is performed, if not try to rotate bone in opposite direction
    if (1 - rootRotation.w < EPSILON)
    {
        rootRotation                = CalculateRootRotation(rawAngles.chord - rawAngles.root, chainData, z, baseBone);
    }

    // Rotate whole chain according to root rotation to calculate relative tip rotation angle.
    Vector newRoot                  = (rootRotation * y) * lengthRoot.l;
    Vector currentTip               = rootRotation * glm::normalize(tip);
    // New tip calculated as a look-at target from new root to the tip
    Vector newTip                   = glm::normalize(target - newRoot);
    // Apply constraints to child rotation
    auto tipRotationParams          = Helpers::CalculateParameters(currentTip, newTip);
    Quaternion tipRotation          = glm::angleAxis(tipRotationParams.angle * constraint.flexibility, tipRotationParams.axis);
    
    // Update new root rotation according to limitations
    chainData.cumulativeRotation    = rootRotation * chainData.cumulativeRotation;
    // Calculate relative rotation of the current bone according to the orienation of its parent bone
    auto parentOrientation          = chainData.cumulativeRotation * parent.GetGlobalOrientation();
    auto childOrientation           = chainData.cumulativeRotation * bone.GetGlobalOrientation();
    
    // Applying constraints for the child bone
    //tipRotation                     = glm::slerp(glm::identity<Quaternion>(), tipRotation, constraint.flexibility);
    auto childRotation              = tipRotation * childOrientation;
    //childRotation                   = bone.ApplyConstraint(glm::inverse(parentOrientation), childRotation);
    childRotation                   = bone.CalculateConstraintRotation(glm::inverse(parentOrientation) * childRotation);
    bone.SetRotation(childRotation); 

    // recalculate tip rotation and target position according to constraints of the child bone
    tipRotation                     = parentOrientation * childRotation * glm::inverse(childOrientation);
    newTip                          = tipRotation * currentTip;

    chainData.tip                   = (newTip * lengthTip.l + (rootRotation * y) * lengthRoot.l);
}

Solver::JointAngles Solver::CalculateAngles(const Length& root, const Length& tip, Vector2 chord) const
{
    JointAngles angles;
    // according to algorithm, x cannot be negative, but it is possible due to FP error,
    // assuming that algorithm is correct with faith in our harts enforce x to 0 and hope that it will not spoil the result
    chord.x                 = std::max(chord.x, 0.0);
    
    // 1st part of the rule of triangle x < y + z
    real chordLength        = glm::clamp(glm::length(chord), root.l - tip.l, root.l + tip.l);
    real lbsq               = chordLength * chordLength;
    // according to the article, calculate position of bones on the coordinate system, 
    // https://www.learnaboutrobots.com/inverseKinematics.htm

    // Angle between chord and X axis
    angles.chord            = (chord.x > EPSILON) ? glm::atan(chord.y/chord.x) : glm::sign(chord.y) * (glm::pi<real>() / 2.0);
     // Angle between X axis and the new direction of the root
    angles.root             = lbsq > EPSILON ? glm::acos(glm::clamp((root.l2 - tip.l2 + lbsq) / (2 * root.l * chordLength), (real)-1., (real)1.)) : 0;

    return angles;    
}

Quaternion Solver::CalculateRootRotation(real angleRoot, const ChainData& chainData, const Vector& z, const Bone& baseBone)
{
    // Calculate modifications for the chain root
    Quaternion rootRotation         = glm::angleAxis((glm::pi<real>() / (real)2.0) - angleRoot, z); 

    // Calculate full rotation of the root bone according to all available root constraints
    Quaternion baseRootAngle        = chainData.cumulativeRotation * chainData.rootRotation;
    rootRotation                    = glm::slerp(glm::identity<Quaternion>(), rootRotation, baseBone.GetConstraints().flexibility);
    //rootRotation                    = baseBone.ApplyConstraint(glm::identity<Quaternion>(), rootRotation * baseRootAngle);
    rootRotation                    = baseBone.CalculateConstraintRotation(rootRotation * baseRootAngle);

    return rootRotation * glm::inverse(baseRootAngle);
}

}
 