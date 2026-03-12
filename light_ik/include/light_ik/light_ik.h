#pragma once
#include <../headers/types.h>
#include <../headers/helpers.h>
#include <memory>

namespace LightIK
{

class Skeleton;
class Solver;

class LightIK
{
public:
    /// @brief Constructs the Light IK plugin
    /// @param bonesCount - total number of bones inside the skeleton
    LightIK(size_t bonesCount);
    ~LightIK();

    /// @brief Restores the default position of the skeleton
    void Reset();

    /// @brief Creates IK chain out of the root chain
    /// @param rootChainDesc - the chain, started from the skeleton root, till the tip of the current chain
    /// @param chainStartIndex - index of the bone from which the actual IK chain is starting
    /// @return index of the created chain
    size_t CreateIKChain(const std::vector<BoneDesc>& rootChainDesc, int chainStartIndex);

    void CompleteChain();
    void SetConstraint(size_t boneIndex, Constraints && constrinat);
    
    void SetRootPosition(const Vector& rootPosition);
    void SetTargetPosition(size_t chainIndex, const Vector& targetPosition);

    // perform required number of backward/forward iteration steps
    size_t UpdateChains(size_t iterations = 1);

    void AddBone(real length, const Quaternion& orientation);
    // complete the rotation chain, by building global orientations of each bone
    // returns quaternions - rotation of each bone in its parent bone local coordinate system
    const std::vector<const Quaternion*> &GetDeltaRotations(size_t chainIndex);

    // functions to support tests
    Vector GetRootPosition(size_t chainIndex) const;
    Vector GetTargetPosition(size_t chainIndex) const;
    real   GetBoneLength(size_t index) const;

private:
    std::unique_ptr<Skeleton> m_skeleton;
    std::vector<std::reference_wrapper<Solver>> m_solvers;
    std::vector<const Quaternion*> m_relativeRotations;
};

}