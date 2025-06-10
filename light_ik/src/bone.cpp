/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "bone.h"
#include "helpers.h"
#include "glm/gtx/norm.inl"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/rotate_vector.hpp"

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

    m_rotationMatrix    = glm::identity<Matrix>();

    m_axis              = glm::normalize(direction);
    m_currentAxis       = m_axis;
    m_front             = Helpers::Normal(m_axis, previousAxis);
    m_root              = boneRoot;
    
    m_length.base       = glm::length(direction);
    m_length.l          = m_length.base;
    m_length.l2         = glm::length2(direction);
}

void Bone::Rotate(const Vector& axis, float angle)
{
    m_rotationMatrix = glm::rotate(m_rotationMatrix, angle, axis);
    m_currentAxis = m_rotationMatrix * Vector4(m_axis, 1);
}

void Bone::Rotate(const Matrix& rotation)
{
    m_rotationMatrix *= rotation;
    m_currentAxis = m_rotationMatrix * Vector4(m_axis, 1);
}

}
