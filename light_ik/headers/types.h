#pragma once
#include <vector>
#include <glm/glm.hpp>

namespace LightIK
{
using real          = double;
using Vector        = glm::vec<3, real, glm::highp>;
using Vector2       = glm::vec<2, real, glm::highp>;
using Vector4       = glm::vec<4, real, glm::highp>;
using Matrix        = glm::mat<3, 3, real, glm::highp>;
using Matrix3       = glm::mat<3, 3, real, glm::highp>;
using Quaternion    = glm::qua<real, glm::highp>;

static const real DELTA = 0.00001f;

constexpr bool EnableDebugLogging = true;

struct RotationParameters
{
    Vector axis{0,0,0};
    real angle{0.f};
};

struct Angle
{
    real current   = 0.f;    // current angle of the bone
    real stiffness = 0.f;    // percentage of the angle acceptance for 1 iteration if 0, bone will try to acheive max available angle
    real min       = -180.f; // if limited, min possible angle for the bone
    real max       = 180.f;  // if limited, max possible angle for the bone
    bool limited   = false;  // apply limits to the bone rotation
    bool locked    = false;  // locks rotation of the bone.
};

struct Length
{
    real base      = 1.f;       // initial length
    real l         = 1.f;       // current length
    real l2        = 1.f;       // current sqared length
    real stretch   = 0.f;       // extension factor. if 0 bone has fixed length
};

}
