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

Bone::Bone(real length, const Quaternion& orientation)
    : m_rotation(orientation)
    , m_length(length, false)
{
    m_globalOrientation = glm::identity<Quaternion>();
}

void Bone::SetRotation(const Quaternion& orientation)
{
    // rotate the bone coordinate system according to delta rotation of the bone
    m_rotation       = orientation;
}

void Bone::SetGlobalOrientation(const Quaternion& orientation)
{
    m_globalOrientation = orientation;
}

}
