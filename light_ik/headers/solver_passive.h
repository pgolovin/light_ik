#pragma once
#include "types.h"
#include "bone.h"
#include "solver_base.h"

namespace LightIK
{

// Solver class
class SolverPassive final : public SolverBase
{
    const size_t m_defaultPose = 0;
    const Vector m_zero{0,0,0};
    const Bone   m_defaultBone{};
public:
    SolverPassive() = default;
    virtual ~SolverPassive() = default;

    const BoneSubchain& GetChain() const                    { return m_chain; }

    size_t GetChainSize() const override                    { return 0; }

    void   SetTipPosition(Vector& position) override        { }
    Vector GetTipPosition() const override                  { return Vector(0, 0, 0); }
    const Vector& GetTargetPosition() const override        { return m_zero; }

    Vector GetRootPosition() const override                 { return Vector(0, 0, 0); }

    void   SetDependencies(bool hasDependencies) override   { }
    bool   HasDependencies() const override                 { return false;}

    bool   TargetReached() const override                   { return true; }
    void   Execute() override                               { }
    
private:
    BoneSubchain            m_chain;
};

}
