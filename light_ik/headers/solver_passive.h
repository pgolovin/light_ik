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
    const Bone   m_defaultBone{};
public:
    SolverPassive(const Bone& baseBone) : m_baseBone(baseBone) {};
    virtual ~SolverPassive() = default;

    const BoneSubchain& GetChain() const                    { return m_chain; }

    size_t GetChainSize() const                             { return 0; }

    void   SetTipPosition(Vector& position) override        { }
    Vector GetTipPosition() const                           { return Vector(0, 0, 0); }

    Vector GetRootPosition() const                          { return Vector(0, 0, 0); }
    const Bone& GetBaseBone() const override                { return m_baseBone; }

    void   SetDependencies(bool hasDependencies) override   { }
    bool   HasDependencies() const override                 { return false;}

    bool   TargetReached() const override                   { return true; }
    void   Execute() override                               { }
    
private:
    const Bone&             m_baseBone;
    BoneSubchain            m_chain;
};

}
