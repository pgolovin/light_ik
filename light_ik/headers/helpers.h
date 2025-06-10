#pragma once
#include "types.h"
#include <glm/glm.hpp>

namespace LightIK
{
    
class Helpers
{
public:
    static Vector Normal(const Vector& axis1, const Vector& axis2);
};

}
