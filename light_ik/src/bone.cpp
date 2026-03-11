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

Bone::Bone(int chainIndex, bool isChained, real length, const Quaternion& orientation)
    : m_rotation(orientation)
    , m_length(length, false)
    , m_index(chainIndex)
    , m_inChain(isChained)
{
    m_globalOrientation         = glm::identity<Quaternion>();
}

void Bone::SetRotation(const Quaternion& orientation)
{
    // relative rotation according to the parent orientation
    m_rotation                  = orientation;
}

void Bone::SetGlobalOrientation(const Quaternion& orientation)
{
    // skeleton global orientation
    m_globalOrientation         = orientation;
}

void Bone::SetConstraints(Constraints && newConstraints)
{
    m_constraints               = std::move(newConstraints);
}

Quaternion Bone::ApplyConstraint(const Quaternion& rotation) const
{
    Vector angles               = glm::clamp(Helpers::ToEulerXZY(rotation), m_constraints.minAngles, m_constraints.maxAngles);
    return Helpers::FromEulerXZY(angles);
}

}
