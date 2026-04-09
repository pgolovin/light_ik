#pragma once
#include "types.h"
#include "bone.h"
#include "target.h"
#include "solver_base.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#include <vector>

namespace LightIK
{

class Skeleton
{
public:
    /// @brief Constructs skeleton 
    /// @param bonesCount total number of bones in the skeleton
    Skeleton(size_t bonesCount);

    /// @brief Create solver for the bone chain.
    /// @param rootChain The root chain is the list of bones from the current chain tip to the skeleton root bone.
    /// @param startBoneIndex Index of the bone from which the IK chain starts
    /// @param target The target model for the chain, it can be either coordinates or bone inside the skeleton
    /// @return reference to the created IK solver
    SolverBase& AddSolver(const std::vector<BoneDesc>& rootChain, size_t startBoneIndex, Target& target);

    /// @brief Adds specific bone chain for monitoring, bones of the chain can be used as internal targets for other skeleton chains.
    /// @param rootChain The root chain is the list of bones from the current chain tip to the skeleton root bone.
    /// @return pointer to the created dummy IK solver, or nullptr if chain was not created
    SolverBase* AddChain(const std::vector<BoneDesc>& rootChain);

    /// @brief Removes IK chain
    /// @param solver the solver assotiated with IK chain that will be removed
    void RemoveSolver(const SolverRef& solver);
    
    /// @brief Assigns constraint to a particular bone of the skeleton
    /// @param boneIndex index of the bone that will have constraints assigned
    /// @param constraint the structure with rotation constraints
    /// @return success if bone registered in any IK chain
    bool SetConstraint(int boneIndex, Constraints && constraint);

    /// @brief Returns the number of created IK chains inside the current skeleton
    /// @return number of IK chains
    size_t GetSolversCount() const                                  { return m_chains.size();      }

    /// @brief Executes all IK mechanics for all chains to reach assotiated target positions
    ///        execution priority equals to the order of chains in the cahin list
    /// @param iterations maximum number of iterrations required to move chains to final position (unused)
    /// @return maximum number of iterrations required to complete chain
    size_t Update(size_t iterations);

    // --------------------------------------------------------------------------------------------------
    // Validation functions should not be used directly inside application
    // --------------------------------------------------------------------------------------------------
    
    /// @brief Forcibly recalculates chain bone positions
    void FinalizeChains();

    /// @brief Returns the array of bones that represents the full chain from local/global root bone till 
    ///        the tip of current chain
    /// @param solver solver the chain is assotiated with
    /// @return vector of bones that represents the root chain
    const std::vector<BoneRef>& GetRootChain(const SolverBase& solver) const;

    /// @brief updates then chain joint positions according to local rotation of all IK bones from
    ///         root bone till the tip bone of the current chain
    /// @warning the function uses slow algorithms and ignores chains priorities,
    ///         it should not be used without strong consideration of the use case
    /// @param solver solver that ownes the IK chain
    /// void CompleteChain(Solver& solver);

    /// @brief Returns the full list of bones envolved into IK mechanics
    /// @return the full list of bones
    const std::vector<BonePtr>& GetBones() const                    { return m_bones;               }

    /// @brief Resets skeleton structure, drops all IK chains and solvers
    void ResetIK();

    /// @brief Resets sceleton position to original pose
    void ResetPose();

private:
    // Descriptor for root chain
    struct RootChain
    {
        // The element list of the root chain
        BoneSubchain    chain;
        // The parent  bone of the chain
        BoneRef         baseBone;
        // Solver that controls the chain
        SolverPtr       solver;
    };
    using RootChainPtr = std::unique_ptr<RootChain>;

    // Find the chain index assotiated with a given solver
    size_t FindChainIndex(const SolverBase& solver) const;
    // Add bone to the skeleton structure. 
    std::pair<bool, BoneRef> AddBone(const BoneDesc& description);
    // calculate positions for all bones in the current chain
    Vector CalculateBonePositions(RootChain& chain);
    // All full chains from root items to tip of the current chain
    std::vector<RootChainPtr> m_chains;
    // Full list of bones assigned to IK chains and their root elements
    std::vector<BonePtr>    m_bones;
};


}
