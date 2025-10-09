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

    void   AddBone(const Vector& boneEnd);
    Bone&  GetBone(size_t index);

    size_t GetChainSize() const { return m_poses[m_defaultPose].bones.size();}
    Vector GetTipPosition() const;

    void   IterateBack();
    void   IterateFront();

    // TODO: Think twice and... remove?
    void   OverrideRootPosition(const Vector& rootPosition);
    Vector GetRootPosition() const { return m_root; }

    void   SetTargetPosition(const Vector& target);
    Vector GetTargetPosition() const { return m_target; }
    
private:
    void   LookAt(Bone& lookAtBone);
    void   ValidateRotationMatrix(const Matrix& rotation, const Vector& testVector) const;

    struct CompoundVector
    {
        Vector direction{0, 0, 0};
        real length2       = 0.f;
        real length        = 0.f;
    };
    std::pair<CompoundVector, CompoundVector>   FormBinaryJoint(const Vector& joint) const;
    Quaternion                                  SolveBinaryJoint(CompoundVector& root, CompoundVector& tip, const Vector& target);
    std::pair<real, real>                       CalculateAngles(const Solver::CompoundVector& root, const Solver::CompoundVector& tip, Vector2 chord) const;

    Pose                m_current; // current position of bones
    std::vector<Pose>   m_poses;   // set of predefined bone positions
    const Bone*         m_lastBone = &m_defaultBone; // helper

    Vector              m_root      {0.f, 0.f, 0.f};
    Vector              m_chainTip  {0.f, 0.f, 0.f};
    Vector              m_target    {0.f, 0.f, 0.f};
    Quaternion          m_cumulativeRotation;

    Vector              m_testRoot  {0.f, 0.f, 0.f};
};

}
