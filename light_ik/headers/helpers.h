#pragma once
#include "types.h"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

namespace LightIK
{
    
class Helpers
{
public:
    static Vector Normal(const Vector& axis1, const Vector& axis2);
    static Quaternion CalculateRotation(const Vector& from, const Vector& to);
    static constexpr Vector DefaultAxis() {return {0,1,0}; }
};

}
