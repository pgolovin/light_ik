/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "helpers.h"
#include "skeleton.h"

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
 