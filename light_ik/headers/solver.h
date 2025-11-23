#pragma once
#include "types.h"
#include "bone.h"

#include <vector>
#include <utility>

namespace LightIK
{

class Solver
{
    const size_t m_defaultPose = 0;
    const Bone   m_defaultBone{};
public:
    Solver();
    virtual ~Solver() {};

    void   AddBone(real length, const Quaternion& localOrientation);
    Bone&  GetBone(size_t index);
    void   CompleteChain();

    size_t GetChainSize() const { return m_poses[m_defaultPose].bones.size();}
    Vector GetTipPosition() const;

    const std::vector<Vector> GetJoints() const {return m_joints;}

    void   IterateBack();
    void   IterateFront();

    // TODO: Think twice and... remove?
    void   OverrideRootPosition(const Vector& rootPosition);
    Vector GetRootPosition() const { return m_joints.front(); }

    void   SetTargetPosition(const Vector& target);
    Vector GetTargetPosition() const { return m_target; }
    
private:
    void                    LookAt(const Vector& initialDirection, const Vector& target);
    Vector                  SolveBinaryJoint(Bone& bone, const Vector& root, const Vector& tip, const Vector& target);
    std::pair<real, real>   CalculateAngles(const Length& root, const Length& tip, Vector2 chord) const;
    
    void   ValidateRotationMatrix(const Matrix& rotation, const Vector& testVector) const;

    Pose                m_current; // current position of bones
    std::vector<Pose>   m_poses;   // set of predefined bone positions
    const Bone*         m_lastBone = &m_defaultBone; // helper

    Vector              m_root      {0.f, 0.f, 0.f};
    Vector              m_chainTip  {0.f, 0.f, 0.f};
    Vector              m_target    {0.f, 0.f, 0.f};
    Quaternion          m_cumulativeRotation;
    Vector              m_testRoot  {0.f, 0.f, 0.f};
    CoordinateSystem    m_lastCoordinateSystem;
    
    Quaternion          m_localOrientation;
    std::vector<Vector> m_joints;
};

}
