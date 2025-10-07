#pragma once
#include "types.h"
#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

namespace LightIK
{

class Bone
{
public:
    Bone() {};
    Bone(const Vector& boneRoot, const Vector& boneTip, const Vector& previousAxis); 

    void SetRotation(const Quaternion& rotation);
    Quaternion Rotate(Quaternion externalRotation);
    const Quaternion& GetRotation() const;

    Quaternion GetChainRotation(const Vector& parentBone) const;

    const Vector& GetRoot() const       {return m_root;}
    const Vector& GetAxis() const       {return m_axis;}
    const Vector& GetFront() const      {return m_front;}
    
    const real GetLength() const        {return m_length.l;}
    const real GetLength2() const       {return m_length.l2;}

    void ReRoot(const Vector& boneRoot) {m_root = boneRoot;}

private:
    void CalculateRotationParameters(const Matrix& localCoordinates, const Matrix& coordinatesTransform);

    Length m_length;

    Vector m_root {0.f, 0.f, 0.f};          // the base point of the bone

    Vector m_axis {0.f, 1.f, 0.f};          // the value is default, and constant, doesn't affected by bone rotations
    Vector m_front{1.f, 0.f, 0.f};          // x is defined as a normal between previous and current bone.

    Quaternion m_rotation;
};

struct Pose
{
    std::vector<Bone>   bones;    
};

}
