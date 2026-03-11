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

std::vector<BoneDesc> LightIKTestBody::ConstructDescriptors(const std::vector<Vector>& chain)
{
    Vector direction{Helpers::DefaultAxis()};
    Vector lastPosition = { 0, 0, 0 };

    // form list of descriptors to build the bone chain upon it
    std::vector<BoneDesc> descriptors;

    for (size_t i = 0; i < chain.size(); i++)
    {
        // build globally oriented bone using previous bone global orientation and local rotation
        Vector axis         = chain[i] - lastPosition;
        real length         = glm::length(axis);
        axis                = glm::normalize(axis);
        Vector rotationAxis = Helpers::Normal(direction, axis);
        real angle          = glm::orientedAngle(direction, axis, rotationAxis);
        std::swap(direction, axis);
        lastPosition = chain[i];

        // place bones in reversed direction from tip to root
        descriptors.emplace_back(BoneDesc{glm::angleAxis(angle, rotationAxis), length, (int)i});
    }      
    return descriptors; 
}

Solver& LightIKTestBody::AddSolver(const std::vector<Vector>& chain, size_t startIndex)
{
    // add solver based on provided descriptions of the bones, 
    // IK chain is build on a part of the chain starting from startIndex (bone index)
    auto descriptors = ConstructDescriptors(chain);
    return m_skeleton->AddSolver(descriptors, startIndex);
}

Skeleton& LightIKTestBody::GetSkeleton() 
{
    return *m_skeleton;
}

};
