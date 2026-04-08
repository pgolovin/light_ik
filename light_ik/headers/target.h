#pragma once
#include "types.h"

namespace LightIK
{

class Bone;
class Skeleton;

/// @brief the interface that represents target of the IK chain
struct Target
{
    virtual const Vector& GetPosition() const = 0;
};
using TargetPtr = std::unique_ptr<Target>;
using TargetRef = std::reference_wrapper<Target>;

// Target that is represented by a simple point in the 3D space
class TargetPosition final : public Target 
{
public:
    TargetPosition() = default;
    TargetPosition(const Vector& target) : m_target(target) {}
    const Vector& GetPosition() const        override  { return m_target;}
    void SetPosition(const Vector& position)           { m_target = position; }
private:
    Vector m_target{0, 0, 0};
};

// Target that is represented by a bone, for internal movements
class TargetBone final : public Target 
{
public:
    TargetBone(Skeleton& skeleton);
    void AssignBone(int boneIndex);
    const Vector& GetPosition() const override;

private:
    Bone* m_target = nullptr;
    Skeleton& m_skeleton;
};

}
