#pragma once

#include "light_ik/light_ik.h"
#include "../../light_ik/headers/skeleton.h"
#include "../../light_ik/headers/solver_base.h"
#include "../../light_ik/headers/solver.h"
#include "../../light_ik/headers/helpers.h"

namespace LightIK
{

class LightIKTestBody
{
public: 
    LightIKTestBody();

    BoneDesc ConstructDescriptor(const Vector& parentOrientation, const Vector& position, const Vector& target, int index);
    // Construct descriptors to build chains upon it
    std::vector<BoneDesc> ConstructDescriptors(const std::vector<Vector>& chain);

    // Construct skeleton bones
    std::vector<BoneDesc> ConstructSkeleton(const std::vector<std::vector<Vector>>& skeleton); 
    
    // Add IK chain
    SolverBase& AddSolver(const std::vector<Vector>& chain, size_t startIndex, Target& target);

    std::vector<SolverRef> CreateSolvers(
        const std::vector<BoneDesc>& skeleton, 
        const std::vector<std::vector<int>>& branches, 
        const std::vector<int>& startIndices, 
        const std::vector<TargetRef>& targets);
    SolverRef CreateSolver(const std::vector<BoneDesc>& skeleton, const std::vector<int>& branches, int startIndices, Target& target);
    SolverRef CreatePassiveChain(const std::vector<BoneDesc>& skeleton, const std::vector<int>& branches);
    // Force chain completion: to calculate joint global positions
    void CompleteChain(SolverBase& solver);

protected:
    Skeleton& GetSkeleton(); 

private:
    std::unique_ptr<Skeleton> m_skeleton;
};

}
