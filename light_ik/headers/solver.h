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
    Pose&  GetBones(size_t poseIndex = 0);
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

    bool   SetConstraint(size_t boneIndex, Constraints&& constraint);
    
private:
    void                    LookAt(const Vector& initialDirection, const Vector& target);
    Vector                  SolveBinaryJoint(Bone& bone, const Vector& root, const Vector& tip, const Vector& target);
    std::pair<real, real>   CalculateAngles(const Length& root, const Length& tip, Vector2 chord) const;
    
    void   ValidateRotationMatrix(const Matrix& rotation, const Vector& testVector) const;

    std::vector<Pose>        m_poses;   // set of predefined bone positions

    Vector                   m_target    {0.f, 0.f, 0.f};
    Quaternion               m_cumulativeRotation;
    Quaternion               m_localOrientation;
    std::vector<Vector>      m_joints;
};

}
