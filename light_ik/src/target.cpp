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

TargetBone::TargetBone(Skeleton& skeleton)
    : m_skeleton(skeleton)
{

}

const Vector& TargetBone::GetPosition() const
{
    assert(m_target); 
    return m_target->GetPosition(); 
}

void TargetBone::AssignBone(int boneIndex) 
{
    m_target = m_skeleton.GetBones().at(boneIndex).get();
}

}
 