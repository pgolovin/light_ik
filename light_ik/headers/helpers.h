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
    static constexpr Vector     DefaultAxis()                   { return {0,1,0}; }
    static real                 Grad2Rad(real grad)             { return grad/180.*glm::pi<real>(); }

    static void                 Print(const std::string& prefix, const Vector& value);
    static void                 Print(const std::string& prefix, const Quaternion& value);
    static void                 Print(const std::string& prefix, const Matrix& value);
};

struct ConstraintSolver
{
    virtual Vector      ToTaitBriant(const Quaternion& q)   = 0;
    virtual Quaternion  FromTaitBriant(const Vector& a)     = 0;
};

struct ConstraintSolverXZY final : public ConstraintSolver
{
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

struct ConstraintSolverZXY final : public ConstraintSolver
{
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

struct ConstraintSolverYXZ final : public ConstraintSolver
{
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

}
