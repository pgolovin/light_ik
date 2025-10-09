/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "bone.h"
#include "helpers.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{

Bone::Bone(const Vector& boneRoot, const Vector& boneTip, const Vector& previousAxis)
{
    Vector direction = boneTip - boneRoot;
    if (glm::length2(direction) < DELTA)
    {
        assert(false);
        throw "Invalid bone length";
    }

    m_axis              = glm::normalize(direction);
    m_front             = Helpers::Normal(m_axis, previousAxis);
    m_root              = boneRoot;

    m_length.l2         = glm::length2(direction);
    m_length.base       = glm::sqrt(m_length.l2);
    m_length.l          = m_length.base;

    m_rotation          = glm::identity<Quaternion>();
}

void Bone::SetRotation(const Quaternion& rotation)
{
    m_rotation = rotation;
}

Quaternion Bone::Rotate(Quaternion externalRotation)
{
    // calculate cumulative rotation from the root to the current bone
    externalRotation    = glm::normalize(m_rotation * externalRotation);
    // rotate axis to get its global rotation
    m_axis              = externalRotation * m_axis;
    // return the rotation for the next bone.
    return externalRotation;
}

Quaternion Bone::GetChainRotation(const Vector& parentBone) const
{
    return Helpers::CalculateRotation(parentBone, m_axis);
}

}
