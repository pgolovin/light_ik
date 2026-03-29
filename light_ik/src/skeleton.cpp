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
    m_bones.resize(bonesCount);
}

SolverBase& Skeleton::AddSolver(const std::vector<BoneDesc>& rootChain, size_t startBoneIndex, Target& target)
{
    // FIXME: ASSUMPTION that the root of the skeleton is and ephimerial bone with index 0
    assert(rootChain.size() && 0 == rootChain.front().boneIndex);
    
    int chainIndex = m_chains.size();
    // Each solver controls specific IK chain
    auto& newChain = m_chains.emplace_back();
    newChain.reserve(rootChain.size());
    std::vector<BoneRef> solverChain;
    solverChain.reserve(rootChain.size());

    // by default all bones after the start Bone Index forms the IK chain
    bool inChain = true;

    // add bones in reverse order from tip to root
    BoneRef baseBone = std::ref(g_boneDummy);
    for (size_t i = rootChain.size(); i != 0; --i)
    {
        size_t index = i - 1;
        auto boneCreation = AddBone(rootChain[index]);

        // verify that bone is still in chain and not a part of any existing chain
        // if it is, and current bone is not a part of IK chain, then the local root bone found
        if (!inChain && !boneCreation.first)
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
        newChain.emplace_back(boneCreation.second);
        if (inChain)
        {
            solverChain.emplace_back(boneCreation.second);
        }
        // if bone index is equal to start bone index of the chain, it means that all previous bones
        // are part of the root chain but not a part of IK calculations
        if (rootChain[index].boneIndex == startBoneIndex)
        {
            inChain = false;
        }
        
    }
    // reverse chain into stright direction
    std::reverse(newChain.begin(), newChain.end());
    std::reverse(solverChain.begin(), solverChain.end());

    // Calculate bone positions for all chain
    CalculateBonePositions(newChain, baseBone);
    
    return *m_solvers.emplace_back(std::make_unique<Solver>(std::move(solverChain), target, baseBone));
}

SolverBase& Skeleton::AddChain(const std::vector<BoneDesc>& rootChain)
{
    assert(rootChain.size() && 0 == rootChain.front().boneIndex);
    
    int chainIndex = m_chains.size();
    auto& newChain = m_chains.emplace_back();
    newChain.reserve(rootChain.size());
    BoneRef baseBone = std::ref(g_boneDummy);
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
        newChain.emplace_back(boneCreation.second);
    }
    // reverse chain into stright direction
    std::reverse(newChain.begin(), newChain.end());

    // Calculate bone positions for all chain
    CalculateBonePositions(newChain, baseBone);
    return *m_solvers.emplace_back(std::make_unique<SolverPassive>(baseBone));
}

void Skeleton::RemoveSolver(const SolverRef& solver)
{
    size_t index = FindChainIndex(solver);

    m_solvers[index] = nullptr;
    m_chains[index].clear();
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
        SolverBase& solver  = *m_solvers[c];
        BoneSubchain& chain = m_chains[c];
        // do the iterrations untill tip and target will be in the same position
        for(size_t i = 0; i < iterations; ++i)
        {
            Vector tip = CalculateBonePositions(chain, solver.GetBaseBone());
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
            Vector tip = CalculateBonePositions(chain, solver.GetBaseBone());
            solver.SetTipPosition(tip);
        }
    }
    return count;
}

void Skeleton::FinalizeChains()
{
    for (size_t c = 0; c < m_chains.size(); ++c)
    {
        Vector tip = CalculateBonePositions(m_chains[c], m_solvers[c]->GetBaseBone());
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

    return m_chains[index]; 
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

Vector Skeleton::CalculateBonePositions(BoneSubchain& chain, const Bone& baseBone)
{   
    // Chain must have at least one bone
    assert(chain.size() > 0);
    // Front kinematics: separated from the solver to make the functionality common and independent from any solvers 
    // Front kinematic always calculated from the chain root position - the bone that either root of overall skeleton,
    //  or bone of the parent IK chain
    Quaternion rotation                 = baseBone.GetGlobalOrientation();
    Vector position                     = baseBone.GetPosition()+ (rotation * Helpers::DefaultAxis() * baseBone.GetLength());

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

}
