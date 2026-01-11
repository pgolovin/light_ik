#pragma once
#include <vector>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace LightIK
{
using real          = double;
using Vector        = glm::vec<3, real, glm::highp>;
using Vector2       = glm::vec<2, real, glm::highp>;
using Vector4       = glm::vec<4, real, glm::highp>;
using Matrix        = glm::mat<3, 3, real, glm::highp>;
using Matrix3       = glm::mat<3, 3, real, glm::highp>;
using Quaternion    = glm::qua<real, glm::highp>;

static const real EPSILON   = 1e-14;

constexpr bool EnableDebugLogging = true;

struct RotationParameters
{
    Vector axis{0,0,0};
    real angle{0.f};
};

using CoordinateSystem = Matrix;
enum class Axis : size_t
{
    x,
    y,
    z,
    total
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
    Length() = default;

    Length(real inL2) 
        : l2(inL2)
        , l(glm::sqrt(inL2))
        , base(l)
    {

    }

    Length(real inL, bool normal)
        : l2(inL * inL)
        , l(inL)
        , base(inL)
    {

    }

    real l         = 1.f;       // current length
    real l2        = 1.f;       // current sqared length
    real base      = 1.f;       // initial length
    real stretch   = 0.f;       // extension factor. if 0 bone has fixed length
};
    
struct Constraints
{
    real flexibility = 1;
    Vector minAngles {-glm::pi<real>(), -glm::pi<real>(), -glm::pi<real>()};
    Vector maxAngles { glm::pi<real>(),  glm::pi<real>(),  glm::pi<real>()};
};

}
