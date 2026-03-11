#pragma once

#include "light_ik/light_ik.h"
#include "../../light_ik/headers/skeleton.h"
#include "../../light_ik/headers/helpers.h"

namespace LightIK
{

class LightIKTestBody
{
public: 
    LightIKTestBody();

    // Construct descriptors to build chains upon it
    std::vector<BoneDesc> ConstructDescriptors(const std::vector<Vector>& chain);
    
    // Add IK chain
    Solver& AddSolver(const std::vector<Vector>& chain, size_t startIndex);

    // Force chain completion: to calculate joint global positions
    void CompleteChain(Solver& solver);

protected:
    Skeleton& GetSkeleton(); 

private:
    std::unique_ptr<Skeleton> m_skeleton;
};

}
