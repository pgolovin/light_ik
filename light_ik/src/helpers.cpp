/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "helpers.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{
    Vector Helpers::Normal(const Vector& axis1, const Vector& axis2)
    {
#ifndef NDEBUG
        if (glm::length2(axis1) < DELTA || glm::length2(axis2) < DELTA )
        {
            return {0.f, 1.f, 0.f};
        }
#endif
        Vector result   = glm::cross(axis1, axis2);
        // if front is 0 length, try to cross it with arbitrary vector (Z axis)
        if (glm::length2(result) < DELTA)
        {
            result      = glm::cross(axis1, {0.f, 0.f, 1.f});
        }
        // well it is possible that Z is aligned with the front , then choose another arbitraty vector (Y)
        if (glm::length2(result) < DELTA)
        {
            result      = glm::cross(axis1, {0.f, 1.f, 0.f});
        }        
        // here it cannot be zero, coz Y and Z are ortogonal, so front must have angle at least with one of them
#ifndef NDEBUG      
        if (glm::length2(result) < DELTA)
        {
            assert(false);
        }
#endif
        return glm::normalize(result);
    }

    Quaternion Helpers::CalculateRotation(const Vector& from, const Vector& to)
    {
        // if from and to vectors are colinear, return zero rotation
        // without this fix error can appear in calculation of close to zero angles that actually should be zero
        if (glm::length2(glm::cross(from, to)) < DELTA)
        {
            return glm::identity<Quaternion>();
        }
        // calculate the rotation axis between the from and to vectors
        Vector rotationAxis = Helpers::Normal(from, to);
        // calculate angle around calculated axis
        real rotationAngle  = glm::orientedAngle(from, to, rotationAxis);
        // form the quaternion to reflect the rotation
        return glm::angleAxis(rotationAngle, rotationAxis);
    }

}
