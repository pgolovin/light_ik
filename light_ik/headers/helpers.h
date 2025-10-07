#pragma once
#include "types.h"
#include <glm/glm.hpp>

namespace LightIK
{
    
class Helpers
{
public:
    static Vector Normal(const Vector& axis1, const Vector& axis2);
    static RotationParameters CalculateRotation(const Vector& axis1, const Vector& axis2);
    static RotationParameters CalculateRotation(const Vector& axis, const Matrix& rotation);

    static constexpr Vector DefaultAxis() {return {0,1,0}; }
    static constexpr Vector4 DefaultAxis4() {return {0,1,0,0}; }
};

}
