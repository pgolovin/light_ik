#pragma once
#include "types.h"
#include <vector>

namespace LightIK
{

class Bone
{
public:
    Bone() {};
    Bone(const Vector& boneRoot, const Vector& boneTip, const Vector& previousAxis); 

    void Rotate(const Vector& axis, float angle);
    void Rotate(const Matrix& rotation);

    const Matrix& GetRotation() const {return m_rotationMatrix;}

    const Vector& GetRoot() const {return m_root;}
    const Vector& GetAxis() const {return m_currentAxis;}
    const Vector& GetFront() const {return m_front;}
    
    const float GetLength() const {return m_length.l;}
    const float GetLength2() const {return m_length.l2;}
    const float GetAngleX() const {return m_angleX.current;}
    const float GetAngleY() const {return m_angleY.current;}
    const float GetAngleZ() const {return m_angleZ.current;}

    void ReRoot(const Vector& boneRoot) {m_root = boneRoot;}
private:
    Length m_length;

    Angle m_angleX;
    Angle m_angleY;
    Angle m_angleZ;

    Vector m_root {0.f, 0.f, 0.f};          // the base point of the bone

    Vector m_axis {0.f, 0.f, 0.f};          // the value is default, and constant, doesn't affected by bone rotations
    Vector m_currentAxis {0.f, 0.f, 1.f};   // the value is default, and constant, doesn't affected by bone rotations
    Vector m_front{1.f, 0.f, 0.f};          // x is defined as a normal between previous and current bone.

    Matrix m_rotationMatrix;
};

struct Pose
{
    std::vector<Bone>   bones;    
};

}
