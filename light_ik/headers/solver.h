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
    Solver(BoneSubchain&& chain, const Bone& parentBone, size_t pivotIndexInChain, Target& target);
    virtual ~Solver() = default;

    const BoneSubchain& GetChain() const;

    size_t GetChainSize() const                             { return m_chain.size();                }

    size_t GetPivotIndex() const override                   { return m_pivotBoneIndex;              }

    void   SetTipPosition(Vector& position) override;
    Vector GetTipPosition() const;

    const Vector& GetTargetPosition() const override        { return m_target.GetPosition();        }

    Vector GetRootPosition() const;

    void   SetDependencies(bool hasDependencies) override   { m_hasDependencies = hasDependencies;  }
    bool   HasDependencies() const override                 { return m_hasDependencies;             }

    bool   TargetReached() const override;
    void   Execute() override;
    
private:
    struct ChainData
    {
        Vector tip {0,0,0};
        Quaternion cumulativeRotation;
        Quaternion rootRotation;
    };
    ChainData               SolveSubchain(const Bone& parentBone, Bone& rootBone, size_t tail, size_t base);
    void                    LookAt(ChainData& chainData, const Vector& target);
    void                    SolveBinaryJoint(ChainData& chainData, Bone& rootBone, Bone& bone, const Bone& parent, const Vector& root, const Vector& tip, const Vector& target);
    struct JointAngles
    {
        real chord  = 0;
        real root   = 0;
    };

    JointAngles             CalculateAngles(const Length& root, const Length& tip, Vector2 chord) const;
    Quaternion              CalculateRootRotation(real angle, const ChainData& chainData, const Vector& z, const Bone& baseBone);

    const Bone&             m_parentBone;
    size_t                  m_pivotBoneIndex = 0;
    BoneSubchain            m_chain;   // bones chain
    Vector                  m_tipPosition {0.f, 0.f, 0.f};
    Target&                 m_target;
    bool                    m_hasDependencies = false;
};

}
