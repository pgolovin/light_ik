#pragma once
#include "types.h"
#include "bone.h"

#include <vector>
#include <glm/glm.hpp>

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
    const Bone& GetBone(size_t index) const;

    size_t GetChainSize() const { return m_poses[m_defaultPose].bones.size();}
    Vector GetTipPosition() const { return m_chainTip; }

    void   IterateBack();
    void   IterateFront();

    // TODO: Think twice and... remove?
    void   OverrideRootPosition(const Vector& rootPosition);
    Vector GetRootPosition() const { return m_root; }

    void   SetTargetPosition(const Vector& target);
    Vector GetTargetPosition() const { return m_target; }
    
private:
    void   LookAt(Bone& lookAtBone);
    void   SolveFlatJoint(Pose& chain, const Vector& target, size_t jointIndex);

    Pose                m_current; // current position of bones
    std::vector<Pose>   m_poses;   // set of predefined bone positions
    const Bone*         m_lastBone = &m_defaultBone; // helper

    Vector              m_root      {0.f, 0.f, 0.f};
    Vector              m_chainTip  {0.f, 0.f, 0.f};
    Vector              m_target    {0.f, 0.f, 0.f};
    Matrix              m_accumulatedRotation;

    Vector              m_testRoot  {0.f, 0.f, 0.f};
};

}
