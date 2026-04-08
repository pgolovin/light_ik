#pragma once
#include <../headers/types.h>
#include <../headers/target.h>
#include <../headers/helpers.h>
#include <memory>

namespace LightIK
{

class Skeleton;
class SolverBase;


class LightIK
{
public:
    /// @brief Constructs the Light IK plugin
    /// @param bonesCount - total number of bones inside the skeleton
    LightIK(size_t bonesCount);
    ~LightIK();

    /// @brief Restores the default position of the skeleton
    void ResetPose();

    /// @brief Removes IK structure of skeleton
    void Reset();

    /// @brief Creates IK chain out of the root chain
    /// @param rootChainDesc - the chain, started from the skeleton root, till the tip of the current chain
    /// @param chainStartIndex - index of the bone from which the actual IK chain is starting
    /// @param target - the target for current chain, it can be either position or another bone
    /// @return index of the created chain
    size_t CreateIKChain(const std::vector<BoneDesc>& rootChainDesc, int chainStartIndex, Target& target);

    /// @brief Creates IK chain that uses skeleton bone as a target
    /// @param rootChainDesc - the chain, started from the skeleton root, till the tip of the current chain
    /// @param chainStartIndex - index of the bone from which the actual IK chain is starting
    /// @param targetBoneIndex - the index of the bone that the chain is targeting to
    /// @return index of the created chain
    size_t CreateIKLink(const std::vector<BoneDesc>& rootChainDesc, int chainStartIndex, int targetBoneIndex);

    /// @brief Creates passive IK chain that can be used in dependent calculations
    /// @param rootChainDesc - the chain, started from the skeleton root
    void CreatePassiveChain(const std::vector<BoneDesc>& rootChainDesc);

    /// @brief Sets constraint for the specific bone
    /// @param boneIndex - index of the bone to set the constraint
    /// @param constrinat - rotation constraint parameters
    void SetConstraint(size_t boneIndex, Constraints && constrinat);
    
    /// @brief perform required number of backward/forward iteration steps
    /// @param iterations - number of iterations to calculate bones positions
    /// @return actual number of iterations
    size_t Update(size_t iterations = 1);

    /// @brief Returns relative rotations for all registered bones
    /// @return vector of quaternions
    const std::vector<const Quaternion*>& GetDeltaRotations();

    const Vector& GetTargetPosition(size_t chainIndex) const;
    /// @brief Create target object that points on bone internal structure
    /// @return internal target object
    TargetBone& CreateInternalTarget();

    TargetPosition& CreateTarget();

    size_t GetSolversCount() const;

    // functions to support tests
    Vector GetTipPosition(size_t chainIndex) const;
    real   GetBoneLength(size_t index) const;
    Vector GetBonePosition(size_t index) const;

private:
    std::unique_ptr<Skeleton> m_skeleton;
    std::vector<std::reference_wrapper<SolverBase>> m_solvers;
    std::vector<const Quaternion*> m_relativeRotations;
    std::vector<TargetPtr> m_targets;
};

}