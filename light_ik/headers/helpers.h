#pragma once
#include "types.h"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#include <string>

namespace LightIK
{
    
class Helpers
{
public:
    static Vector               Normal(const Vector& axis1, const Vector& axis2);
    static Quaternion           CalculateRotation(const Vector& from, const Vector& to);
    static RotationParameters   CalculateParameters(const Vector& from, const Vector& to);
    static Matrix               CalculateTransferMatrix(const CoordinateSystem& base, const CoordinateSystem& target);
    static Vector               ToLocal(const CoordinateSystem& localSystem, const Vector& vector);
    static constexpr Vector     DefaultAxis() {return {0,1,0}; }

    static void                 Print(const std::string& prefix, const Vector& value);
    static void                 Print(const std::string& prefix, const Quaternion& value);
    static void                 Print(const std::string& prefix, const Matrix& value);
};

}
