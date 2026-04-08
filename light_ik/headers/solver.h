#pragma once
#include "types.h"
#include "target.h"
#include "bone.h"
#include "solver_base.h"

#include <vector>
#include <utility>
#include <functional>

namespace LightIK
{

// Solver class
class Solver final : public SolverBase
{
    const size_t m_defaultPose = 0;
    const Bone   m_defaultBone{};
public:
    Solver(BoneSubchain&& chain, const Bone& parentBone, Target& target);
    virtual ~Solver() = default;

    const BoneSubchain& GetChain() const;

    size_t GetChainSize() const                             { return m_chain.size();}

    void   SetTipPosition(Vector& position) override;
    Vector GetTipPosition() const;

    const Vector& GetTargetPosition() const override        { return m_target.GetPosition(); }

    Vector GetRootPosition() const;

    void   SetDependencies(bool hasDependencies) override   { m_hasDependencies = hasDependencies;}
    bool   HasDependencies() const override                 { return m_hasDependencies;}

    bool   TargetReached() const override;
    void   Execute() override;
    
private:
    void                    LookAt(const Vector& initialDirection, const Vector& target);
    Vector                  SolveBinaryJoint(Bone& bone, const Bone& parent, const Vector& root, const Vector& tip, const Vector& target);
    std::pair<real, real>   CalculateAngles(const Length& root, const Length& tip, Vector2 chord) const;
    
    const Bone&             m_parentBone;
    BoneSubchain            m_chain;   // bones chain
    Vector                  m_tipPosition {0.f, 0.f, 0.f};
    Target&                 m_target;
    Quaternion              m_cumulativeRotation;
    bool                    m_hasDependencies = false;
};

}
