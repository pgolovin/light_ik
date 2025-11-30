#pragma once
#include <../headers/types.h>
#include <../headers/helpers.h>
#include <memory>

namespace LightIK
{

class Solver;

class LightIK
{
public:
    LightIK();
    ~LightIK();

    void Reset();

    void AddBone(real length, const Quaternion& orientation);
    // complete the rotation chain, by building global orientations of each bone
    void CompleteChain();
    void SetConstraint(size_t boneIndex, Constraints && constrinat);
    
    void SetRootPosition(const Vector& rootPosition);
    void SetTargetPosition(const Vector& targetPosition);

    // perform required number of backward/forward iteration steps
    size_t UpdateChainPosition(size_t iterrations = 1);

    // returns quaternions - rotation of each bone in its parent bone local coordinate system
    std::vector<Quaternion> GetDeltaRotations() const;

    // functions to support tests
    Vector GetRootPosition() const;
    Vector GetTargetPosition() const;
    real   GetBoneLength(size_t index) const;

private:
    std::unique_ptr<Solver> m_solver;
};

}