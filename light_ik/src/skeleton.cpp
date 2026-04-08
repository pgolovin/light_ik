/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "skeleton.h"
#include "solver.h"
#include "solver_passive.h"
#include "helpers.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <iostream>

namespace LightIK
{

static Bone g_boneDummy;

Skeleton::Skeleton(size_t bonesCount)
{
    // Add one more bone slot for the root bone, it will be placed in the end of list
    m_bones.resize(bonesCount);
}

SolverBase& Skeleton::AddSolver(const std::vector<BoneDesc>& rootChain, size_t startBoneIndex, Target& target)
{
    // Root bone is not 0, so consider that all root chains are made from tip to root.
    assert(rootChain.size());
    
    int chainIndex = m_chains.size();
    // Each solver controls specific IK chain
    RootChain& newChain = *m_chains.emplace_back(std::make_unique<RootChain>(RootChain{BoneSubchain{}, std::ref(g_boneDummy)}));
    newChain.chain.reserve(rootChain.size());

    std::vector<BoneRef> solverChain;
    solverChain.reserve(rootChain.size());

    // Add bones in reverse order from tip to root
    Bone* parentBone  = &g_boneDummy;
    
    // By default all bones after the start Bone Index forms the IK chain
    bool inChain        = true;
    
    for (size_t i = rootChain.size(); i != 0; --i)
    {
        size_t index = i - 1;
        auto boneCreation = AddBone(rootChain[index]);

        // If the bone is the first bone not in the chain, then the parent bone is found
        if (!inChain && parentBone == &g_boneDummy)
        {
            parentBone = &boneCreation.second.get();
        }

        // Verify that bone is still in chain and not a part of any existing chain
        //  if it is, and current bone is not a part of IK chain, then the local root bone found
        if (!inChain && !boneCreation.first)
        {
            Bone& newBone = boneCreation.second;
            // Mark the owner of the bone that it has dependencies
            if (newBone.GetOwner())
            {
                newBone.GetOwner()->SetDependencies(true);
            }
            newChain.baseBone = std::ref(boneCreation.second);
            break;
        }
        // Add bone to the root chain
        newChain.chain.emplace_back(boneCreation.second);
        // If in chain, add bone to the solver chain
        if (inChain)
        {
            solverChain.emplace_back(boneCreation.second);
        }
        // If bone index is equal to start bone index of the chain, it means that all previous bones
        //  are part of the root chain but not a part of IK calculations
        if (rootChain[index].boneIndex == startBoneIndex)
        {
            // Chain is finished
            inChain = false;
        }
    }
    // reverse chain into stright direction
    std::reverse(newChain.chain.begin(), newChain.chain.end());
    std::reverse(solverChain.begin(), solverChain.end());

    // Calculate bone positions for all chain
    auto tipPosition = CalculateBonePositions(newChain);
    // Add new solver
    assert(parentBone);

    std::unique_ptr<Solver> newSolver = std::make_unique<Solver>(std::move(solverChain), *parentBone, target);
    newSolver->SetTipPosition(tipPosition);
    
    return *m_solvers.emplace_back(std::move(newSolver));
}

SolverBase* Skeleton::AddChain(const std::vector<BoneDesc>& rootChain)
{
    assert(rootChain.size());
    
    int chainIndex = m_chains.size();
    BoneRef baseBone = std::ref(g_boneDummy);
    BoneSubchain chain;
    chain.reserve(rootChain.size());
    for (size_t i = rootChain.size(); i != 0; --i)
    {
        size_t index = i - 1;
        auto boneCreation = AddBone(rootChain[index]);

        // verify that bone is still in chain and not a part of any existing chain
        // if it is, and current bone is not a part of IK chain, then the local root bone found
        if (!boneCreation.first)
        {
            Bone& newBone = boneCreation.second;
            // mark the owner of the bone that it has dependencies
            if (newBone.GetOwner())
            {
                newBone.GetOwner()->SetDependencies(true);
            }
            baseBone = std::ref(boneCreation.second);
            break;
        }
        chain.emplace_back(boneCreation.second);
    }
    // If chain already created or part of another chain, no need to create it
    if (!chain.size())
    {
        return nullptr;
    }

    // Reverse chain into stright direction
    std::reverse(chain.begin(), chain.end());

    // Add new chain only if it has at least one element
    RootChain& newChain = *m_chains.emplace_back(std::make_unique<RootChain>(RootChain{std::move(chain), baseBone}));

    // Calculate bone positions for all chain
    CalculateBonePositions(newChain);
    return m_solvers.emplace_back(std::make_unique<SolverPassive>()).get();
}

void Skeleton::RemoveSolver(const SolverRef& solver)
{
    size_t index = FindChainIndex(solver);

    m_solvers[index]    = nullptr;
    m_chains[index]     = nullptr;
}

bool Skeleton::SetConstraint(int boneIndex, Constraints && constraint)
{
    assert(boneIndex < m_bones.size());

    Bone* bone = m_bones[boneIndex].get();
    if (bone)
    {
        bone->SetConstraints(std::move(constraint));
    }
    return bone;
}

size_t Skeleton::Update(size_t iterations)
{
    size_t count = iterations;
    for (size_t c = 0; c < m_chains.size(); ++c)
    {
        SolverBase& solver      = *m_solvers[c];
        RootChain& rootChain    = *m_chains[c];
        // do the iterrations untill tip and target will be in the same position
        for(size_t i = 0; i < iterations; ++i)
        {
            Vector tip = CalculateBonePositions(rootChain);
            solver.SetTipPosition(tip);

            if (solver.TargetReached())
            {
                // return false if no iterations were done
                count = std::min(count, i);
                break;
            }

            solver.Execute();            
        }
        
        if (solver.HasDependencies())
        {
            Vector tip = CalculateBonePositions(rootChain);
            solver.SetTipPosition(tip);
        }
    }
    return count;
}

void Skeleton::FinalizeChains()
{
    for (size_t c = 0; c < m_chains.size(); ++c)
    {
        Vector tip = CalculateBonePositions(*m_chains[c]);
        m_solvers[c]->SetTipPosition(tip);
    }
}

const std::vector<BoneRef>& Skeleton::GetRootChain(const SolverBase& solver) const    
{ 
    size_t index = FindChainIndex(solver);

    static const std::vector<BoneRef> stub = {};
    if (index >= m_chains.size())
    {
        return stub;
    }

    return m_chains[index]->chain; 
}

size_t Skeleton::FindChainIndex(const SolverBase& solver) const
{
    size_t index = 0;
    for (auto& solverItem : m_solvers)
    {
        if (solverItem.get() == (SolverBase*)&solver)
        {
            return index;
        }
        ++index;
    }
    return -1LLU;
}

std::pair<bool, BoneRef> Skeleton::AddBone(const BoneDesc& description)
{
    // add new bone to the chain
    assert (m_bones.size() > description.boneIndex);

    auto& bone = m_bones[description.boneIndex];
    bool created = !bone;
    // if bone already exists, return the existing bone, otherwise create a new bone and attach it to current chain
    if (!bone)
    {
        bone = std::make_unique<Bone>(description.length, description.orientation);
    }
        
    return {created, *bone};
}

Vector Skeleton::CalculateBonePositions(RootChain& rootChain)
{   
    auto& chain = rootChain.chain;
    // Chain must have at least one bone
    assert(chain.size() > 0);
    // Front kinematics: separated from the solver to make the functionality common and independent from any solvers 
    // Front kinematic always calculated from the chain root position - the bone that either root of overall skeleton,
    //  or bone of the parent IK chain
    Quaternion rotation                 = rootChain.baseBone.get().GetGlobalOrientation();
    Vector position                     = rootChain.baseBone.get().GetPosition()+ (rotation * Helpers::DefaultAxis() * rootChain.baseBone.get().GetLength());

    for (size_t i = 0; i < chain.size(); ++i)
    {
        chain[i].get().SetPosition(position);
        // Calculate cumuilative change of orientation of the current bone
        rotation                        = rotation * chain[i].get().GetRotation();
        chain[i].get().SetGlobalOrientation(rotation);
        // Find the new position of the bone base joint
        position                        = position + (rotation * Helpers::DefaultAxis() * chain[i].get().GetLength());
    }
    return position;
}

void Skeleton::ResetIK()
{
    m_solvers.clear();
    m_chains.clear();
    // reset all created bones to build skeletal structure from scratch
    // TO THINK: suboptimal algorithm
    for (auto& bone : m_bones)
    {
        bone = nullptr;
    }
}

void Skeleton::ResetPose()
{
    for (auto& bone : m_bones)
    {
        if (bone)
        {
            bone->Reset();
        }
    }
    FinalizeChains();
}

}
