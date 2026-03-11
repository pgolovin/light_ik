#pragma once
#include "types.h"
#include "bone.h"
#include "solver.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#include <vector>

namespace LightIK
{

class Skeleton
{
public:
    Skeleton(size_t bonesCount);

    /// @brief Create solver for the bone chain.
    /// @param rootChain The root chain is the list of bones from the current chain tip to the skeleton root bone.
    /// @param startBoneIndex Index of the bone from which the IK chain starts
    /// @return reference to the creates IK solver
    Solver& AddSolver(const std::vector<BoneDesc>& rootChain, size_t startBoneIndex);

    /// @brief Removes IK chain
    /// @param solver the solver assotiated with IK chain that will be removed
    void RemoveSolver(const Solver& solver);
    
    /// @brief Assigns constraint to a particular bone of the skeleton
    /// @param boneIndex index of the bone that will have constraints assigned
    /// @param constraint the structure with rotation constraints
    void SetConstraint(int boneIndex, Constraints && constraint);

    size_t GetChainsCount() const                               { return m_chains.size(); }

    // validation functions should not be used directly inside application
    const std::vector<BoneRef>& GetRootChain(const Solver& solver) const;

    /// @brief updates then chain joint positions according to local rotation of all IK bones from
    ///         root bone till the tip bone of the current chain
    ///
    /// @warning the function uses slow algorithms and ignores chains priorities,
    ///         it should not be used without strong consideration of the use case
    ///
    /// @param solver solver that ownes the IK chain
    void CompleteChain(Solver& solver);

    const std::vector<BonePtr>& GetBones() const {return m_bones;};
private:
    // Find the chain index assotiated with a given solver
    size_t FindChainIndex(const Solver& solver) const;
    // Add bone to the skeleton structure. 
    Bone& AddBone(const BoneDesc& description, int chainIndex, bool inChain);

    // Actual IK chain solvers
    std::vector<SolverPtr>  m_solvers;
    // All full chains from root items to tip of the current chain
    std::vector<std::vector<BoneRef>> m_chains;
    // Full list of bones assigned to IK chains and their root elements
    std::vector<BonePtr>    m_bones;
};


}
