#pragma once
#include <vector>
#include <glm/glm.hpp>

namespace LightIK
{
using Vector = glm::vec3;
using Vector2 = glm::vec2;
using Vector4 = glm::vec4;
using Matrix = glm::mat4x4;
static const float DELTA = 0.00001f;

constexpr bool EnableDebugLogging = true;

struct Angle
{
    float current   = 0.f;    // current angle of the bone
    float stiffness = 0.f;    // percentage of the angle acceptance for 1 iteration if 0, bone will try to acheive max available angle
    float min       = -180.f; // if limited, min possible angle for the bone
    float max       = 180.f;  // if limited, max possible angle for the bone
    bool  limited   = false;  // apply limits to the bone rotation
    bool  locked    = false;  // locks rotation of the bone.
};

struct Length
{
    float base      = 1.f;       // initial length
    float l         = 1.f;       // current length
    float l2        = 1.f;       // current sqared length
    float stretch   = 0.f;       // extension factor. if 0 bone has fixed length
};

}
