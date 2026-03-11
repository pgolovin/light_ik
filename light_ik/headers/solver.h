#pragma once
#include "types.h"
#include "bone.h"

#include <vector>
#include <utility>
#include <functional>

namespace LightIK
{

class Solver
{
    const size_t m_defaultPose = 0;
    const Bone   m_defaultBone{};
public:
    Solver(BoneSubchain&& chain);
    virtual ~Solver() {};

    
    const BoneSubchain& GetChain() const;

    size_t GetChainSize() const { return m_chain.size();}

    void   SetTipPosition(Vector& position);
    Vector GetTipPosition() const;

    void   IterateBack();

    Vector GetRootPosition() const;

    void   SetTargetPosition(const Vector& target);
    Vector GetTargetPosition() const { return m_target; }

    bool   SetConstraint(size_t boneIndex, Constraints&& constraint);
    
private:
    void                    LookAt(const Vector& initialDirection, const Vector& target);
    Vector                  SolveBinaryJoint(Bone& bone, const Bone& parent, const Vector& root, const Vector& tip, const Vector& target);
    std::pair<real, real>   CalculateAngles(const Length& root, const Length& tip, Vector2 chord) const;
    
    BoneSubchain            m_chain;   // bones chain
    Vector                  m_tipPosition {0.f, 0.f, 0.f};
    Vector                  m_target      {0.f, 0.f, 0.f};
    Quaternion              m_cumulativeRotation;
};

using SolverPtr = std::unique_ptr<Solver>;
using SolverRef = std::reference_wrapper<Solver>;

}
