/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "helpers.h"
#include "glm/gtx/norm.inl"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/rotate_vector.hpp"

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
}
