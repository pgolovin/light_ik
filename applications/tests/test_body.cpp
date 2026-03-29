#include <memory>
#include <gtest/gtest.h>

#include "test_body.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/norm.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace LightIK
{

LightIKTestBody::LightIKTestBody()
{
    m_skeleton = std::make_unique<Skeleton>(10); 
}

BoneDesc LightIKTestBody::ConstructDescriptor(const Vector& parentOrientation, const Vector& position, const Vector& target, int index)
{
    Vector axis         = target - position;
    real length         = glm::length(axis);
    axis                = glm::normalize(axis);
    Vector rotationAxis = Helpers::Normal(parentOrientation, axis);
    real angle          = glm::orientedAngle(parentOrientation, axis, rotationAxis);

    return BoneDesc{glm::angleAxis(angle, rotationAxis), length, index};
}

std::vector<BoneDesc> LightIKTestBody::ConstructDescriptors(const std::vector<Vector>& chain)
{
    Vector direction{Helpers::DefaultAxis()};
    Vector lastPosition = { 0, 0, 0 };

    // form list of descriptors to build the bone chain upon it
    std::vector<BoneDesc> descriptors;

    for (size_t i = 0; i < chain.size(); i++)
    {
        // build globally oriented bone using previous bone global orientation and local rotation
        // place bones in reversed direction from tip to root
        descriptors.emplace_back(ConstructDescriptor(direction, lastPosition, chain[i], i));
        
        Vector axis  = glm::normalize(chain[i] - lastPosition);
        std::swap(direction, axis);
        lastPosition = chain[i];
    }      
    return descriptors; 
}

std::vector<BoneDesc> LightIKTestBody::ConstructSkeleton(const std::vector<std::vector<Vector>>& skeleton)
{
    std::vector<BoneDesc> descriptors;
    int index = 0;
    for (auto& chain : skeleton)
    {
        Vector direction = Helpers::DefaultAxis();
        for (size_t i = 0; i < chain.size() - 1; ++i)
        {
            descriptors.emplace_back(ConstructDescriptor(direction, chain[i], chain[i + 1], index));
            direction = glm::normalize(chain[i + 1] - chain[i]);
            ++index;
        }
    }
    return descriptors;
}

SolverBase& LightIKTestBody::AddSolver(const std::vector<Vector>& chain, size_t startIndex, Target& target)
{
    // add solver based on provided descriptions of the bones, 
    // IK chain is build on a part of the chain starting from startIndex (bone index)
    auto descriptors = ConstructDescriptors(chain);
    return m_skeleton->AddSolver(descriptors, startIndex, target);
}

std::vector<SolverRef> LightIKTestBody::CreateSolvers(
    const std::vector<BoneDesc>& skeleton,
    const std::vector<std::vector<int>>& branches, 
    const std::vector<int>& startIndices, 
    const std::vector<TargetRef>& targets)
{
    std::vector<SolverRef> solvers;
    for (size_t c = 0; c < branches.size(); ++c)
    {
        solvers.emplace_back(CreateSolver(skeleton, branches[c], startIndices[c], targets[c]));
    }    
    return solvers;
}

SolverRef LightIKTestBody::CreateSolver(const std::vector<BoneDesc>& skeleton, const std::vector<int>& branches, int startIndex, Target& target)
{
    std::vector<BoneDesc> subDescriptors;
     for (int i : branches)
    {
        subDescriptors.emplace_back(skeleton[i]);
    }
    return std::ref(GetSkeleton().AddSolver(subDescriptors, startIndex, target));
}

SolverRef LightIKTestBody::CreatePassiveChain(const std::vector<BoneDesc>& skeleton, const std::vector<int>& branches)
{
    std::vector<BoneDesc> subDescriptors;

    for (int i : branches)
    {
        subDescriptors.emplace_back(skeleton[i]);
    }
    
    return std::ref(GetSkeleton().AddChain(subDescriptors));
}

Skeleton& LightIKTestBody::GetSkeleton() 
{
    return *m_skeleton;
}

};
