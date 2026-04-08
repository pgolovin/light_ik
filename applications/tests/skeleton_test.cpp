#include <memory>
#include <gtest/gtest.h>

#include "test_helpers.h"
#include "test_body.h"
#include "../../light_ik/headers/skeleton.h"
#include "../../light_ik/headers/helpers.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/norm.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <array>

namespace LightIK
{

TEST(SkeletonTest, can_create_skeleton)
{
    std::unique_ptr<Skeleton> library;
    ASSERT_NO_THROW(library = std::make_unique<Skeleton>(10));
};

class SkeletonBaseTest : public ::testing::Test, public LightIKTestBody
{
};

TEST_F(SkeletonBaseTest, no_default_chains)
{
    ASSERT_EQ(0, GetSkeleton().GetSolversCount());
}

TEST_F(SkeletonBaseTest, new_chain)
{
    TargetPosition target;
    ASSERT_NO_THROW(AddSolver({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}}, 0, target));
}

TEST_F(SkeletonBaseTest, new_chain_added)
{
    TargetPosition target;
    AddSolver({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}}, 0, target);
    ASSERT_EQ(1LLU, GetSkeleton().GetSolversCount());
}

TEST_F(SkeletonBaseTest, passive_chain_created)
{
    TargetPosition target;
    
    auto descriptors = ConstructDescriptors({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}});
    ASSERT_TRUE(GetSkeleton().AddChain(descriptors));
}

TEST_F(SkeletonBaseTest, passive_chain_added)
{
    TargetPosition target;
    
    auto descriptors = ConstructDescriptors({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}});
    GetSkeleton().AddChain(descriptors);
    ASSERT_EQ(1LLU, GetSkeleton().GetSolversCount());
}

TEST_F(SkeletonBaseTest, passive_chain_over_existing)
{
    TargetPosition target;
    auto descriptors = ConstructDescriptors({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}});
    GetSkeleton().AddSolver(descriptors, 0, target);
    ASSERT_FALSE(GetSkeleton().AddChain(descriptors));
}

TEST_F(SkeletonBaseTest, passive_chain_not_added)
{
    TargetPosition target;
    auto descriptors = ConstructDescriptors({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}});
    GetSkeleton().AddSolver(descriptors, 0, target);
    GetSkeleton().AddChain(descriptors);
    ASSERT_EQ(1LLU, GetSkeleton().GetSolversCount());
}

TEST_F(SkeletonBaseTest, reset)
{
    TargetPosition target;
    AddSolver({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}}, 0, target);
    GetSkeleton().ResetIK();
    ASSERT_EQ(0LLU, GetSkeleton().GetSolversCount());
}

TEST_F(SkeletonBaseTest, new_chain_index)
{
    TargetPosition target;
    std::vector<Vector> chain = {Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}};
    SolverBase& solver = AddSolver(chain, 0, target);
}

TEST_F(SkeletonBaseTest, new_chain_size)
{
    TargetPosition target;
    std::vector<Vector> chain = {Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}};
    SolverBase& solver = AddSolver(chain, 0, target);
    ASSERT_EQ(chain.size(), GetSkeleton().GetRootChain(solver).size());
}

TEST_F(SkeletonBaseTest, new_chain_content)
{
    TargetPosition target;
    std::vector<Vector> chain = {Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}};
    SolverBase& solver = AddSolver(chain, 0, target);
    for (size_t i = 0; i < chain.size(); ++i)
    {
        TestHelpers::CompareVectors(chain[i], GetSkeleton().GetRootChain(solver)[i].get().GetPosition());
    }
}

TEST_F(SkeletonBaseTest, partial_ik_chain)
{
    TargetPosition target;
    SolverBase& solver = AddSolver({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}}, 1, target);
    auto& chain = GetSkeleton().GetRootChain(solver);
    ASSERT_FALSE(chain[0].get().GetOwner());
    ASSERT_TRUE(chain[1].get().GetOwner());
    ASSERT_TRUE(chain[2].get().GetOwner());
}

TEST_F(SkeletonBaseTest, add_second_chain)
{
    TargetPosition target;
    std::vector<Vector> chain {Vector{0,0,0}, {0,1,0}, {0,2,0}, {0,3,0}, {0,4,0}};
    AddSolver(chain, 0, target);

    std::vector<Vector> secondaryChain {Vector{0,0,0}, {1,0,0}, {1,1,0}, {1,2,0}};
    auto descriptors = ConstructDescriptors(secondaryChain);
    
    for (size_t i = 0; i < secondaryChain.size(); ++i)
    {
        descriptors[i].boneIndex = 5 + i;
    }
    descriptors.front().boneIndex = 0;

    GetSkeleton().AddSolver(descriptors, 0, target);
    ASSERT_EQ(2, GetSkeleton().GetSolversCount());
}

TEST_F(SkeletonBaseTest, one_bone_chain)
{
    TargetPosition target;
    ASSERT_NO_THROW(GetSkeleton().AddSolver({BoneDesc{glm::identity<Quaternion>(), 1.f, 0}}, 0, target));
}

TEST_F(SkeletonBaseTest, one_bone_chain_size)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({BoneDesc{glm::identity<Quaternion>(), 1.f, 0}}, 0, target);
    ASSERT_EQ(1LLU, solver.GetChainSize());
}

TEST_F(SkeletonBaseTest, construct_chain)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({BoneDesc{glm::identity<Quaternion>(), 1.f, 0}}, 0, target);
    ASSERT_NO_THROW(GetSkeleton().Update(1));
}

TEST_F(SkeletonBaseTest, tip_oriented_position)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0}}, 0, target);
    GetSkeleton().Update(1);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 0, 2), solver.GetTipPosition()));
}

TEST_F(SkeletonBaseTest, two_bone_chain)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::identity<Quaternion>(), 1.f, 1}
    }, 0, target);

    GetSkeleton().Update(1);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 0, 3), solver.GetTipPosition()));
}

TEST_F(SkeletonBaseTest, two_bone_chain_oriented)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 1.f, 1}
    }, 0, target);

    GetSkeleton().Update(1);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, -1, 2), solver.GetTipPosition()));
}

TEST_F(SkeletonBaseTest, two_bone_chain_oriented_3D)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{0,0,1}), 1.f, 1}
    }, 0, target);

    GetSkeleton().Update(1);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(-1, 0, 2), solver.GetTipPosition()));
}

TEST_F(SkeletonBaseTest, independent_solver)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{0,0,1}), 1.f, 1}
    }, 0, target);

    ASSERT_FALSE(solver.HasDependencies());
}

TEST_F(SkeletonBaseTest, bone_positions)
{
    TargetPosition target;
    std::vector<Vector> chain{Vector{0, 1, 0}, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}};
    SolverBase& solver = AddSolver(chain, 0, target);

    GetSkeleton().Update(1);
    const auto& rootChain = GetSkeleton().GetRootChain(solver);

    for(size_t i = 0; i < chain.size() - 1; ++i)
    {
        ASSERT_TRUE(TestHelpers::CompareVectors(chain[i], rootChain[i + 1].get().GetPosition())) << "Failed at " << i << "th";
    }
    ASSERT_TRUE(TestHelpers::CompareVectors(chain.back(), solver.GetTipPosition())) << "Failed at tip";
}

TEST_F(SkeletonBaseTest, chain_movement)
{
    TargetPosition target;
    std::vector<Vector> chain{Vector{0, 1, 0}, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}};
    SolverBase& solver = AddSolver(chain, 0, target);
    target.SetPosition({0, 4, 4});
    GetSkeleton().Update(1);
    GetSkeleton().FinalizeChains();

    const auto& rootChain = GetSkeleton().GetRootChain(solver);
    for(size_t i = 0; i < chain.size() - 1; ++i)
    {
        ASSERT_FALSE(TestHelpers::CompareVectors(chain[i], rootChain[i + 1].get().GetPosition())) << "Failed at " << i << "th";
    }
    ASSERT_FALSE(TestHelpers::CompareVectors(chain.back(), solver.GetTipPosition())) << "Failed at tip";
}

TEST_F(SkeletonBaseTest, bone_reset)
{
    TargetPosition target;
    std::vector<Vector> chain{Vector{0, 1, 0}, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}};
    SolverBase& solver = AddSolver(chain, 0, target);
    target.SetPosition({0, 4, 4});
    GetSkeleton().Update(1);
    GetSkeleton().FinalizeChains();
    GetSkeleton().ResetPose();
    const auto& rootChain = GetSkeleton().GetRootChain(solver);

    for(size_t i = 0; i < chain.size() - 1; ++i)
    {
        ASSERT_TRUE(TestHelpers::CompareVectors(chain[i], rootChain[i + 1].get().GetPosition())) << "Failed at " << i << "th";
    }
    ASSERT_TRUE(TestHelpers::CompareVectors(chain.back(), solver.GetTipPosition())) << "Failed at tip";
}


class SkeletonChainingTest : public SkeletonBaseTest
{
public: 
    std::vector<SolverRef>& ConstructSkeleton(std::vector<int> startIndices)
    {
        std::vector<std::vector<Vector>> bones ={
            /*                                           0          1          2          3          4          */
            /*root chain*/ std::vector<Vector>{Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}},
            /*                                           5          6            */ 
            /*branch 1 */                     {      {0, 2, 0}, {1, 2, 0}, {2, 2, 0}},
            /*                                           7          8            */ 
            /*branch 2 */                     {      {0, 4, 0}, {1, 4, 0}, {2, 4, 0}}
        };

        std::vector<BoneDesc> descriptors = LightIKTestBody::ConstructSkeleton(bones);
        std::vector<std::vector<int>> structure = {
            std::vector<int>{0, 1, 2, 3, 4}, // spine
            {0, 1, 5, 6},                    // branch 1
            {0, 1, 2, 3, 7, 8}               // branch 2
        };

        m_solvers = CreateSolvers(descriptors, structure, startIndices, {m_target, m_target, m_target});
        return m_solvers;
    }

protected:
    TargetPosition m_target;
    std::vector<SolverRef> m_solvers;
};

TEST_F(SkeletonChainingTest, add_branches)
{
    
    auto& solvers = ConstructSkeleton({0, 5, 7});
    ASSERT_EQ(3, GetSkeleton().GetSolversCount());
}


TEST_F(SkeletonChainingTest, skeleton_structure)
{
    std::vector<Vector> bones {
        /*                   0          1          2          3          4  */
        /*r*/ Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0},// {0, 5, 0},
        /*                   5          6     */
        /*branch 1*/{0, 2, 0}, {1, 2, 0},// {2, 2, 0},
        /*                   7          8     */
        /*branch 2*/{0, 4, 0}, {1, 4, 0},// {2, 4, 0}
    };

    TargetPosition target;
    auto& solvers = ConstructSkeleton({0, 5, 7});
    const auto& skeletonBones = GetSkeleton().GetBones();
    for (size_t i = 0; i < bones.size(); ++i)
    {
        ASSERT_TRUE(TestHelpers::CompareVectors(bones[i], skeletonBones[i]->GetPosition())) << " Failed on " << i << "th iteration";
    }
}

TEST_F(SkeletonChainingTest, branch_truncation)
{
    auto& solvers = ConstructSkeleton({0, 5, 7});
    auto& branchChain = GetSkeleton().GetRootChain(solvers[1]);
    ASSERT_EQ(2LLU, branchChain.size());
}

TEST_F(SkeletonChainingTest, truncated_branch_chain)
{
    auto& solvers = ConstructSkeleton({0, 5, 7});
    auto branchChain = solvers[1].get().GetChainSize();
    ASSERT_EQ(2LLU, branchChain);
}

TEST_F(SkeletonChainingTest, solver_with_dependencies)
{
    auto& solvers = ConstructSkeleton({0, 5, 7});
    ASSERT_TRUE(solvers[0].get().HasDependencies());
}

TEST_F(SkeletonChainingTest, independent_branch)
{
    auto& solvers = ConstructSkeleton({0, 5, 7});
    ASSERT_FALSE(solvers[1].get().HasDependencies());
}

TEST_F(SkeletonChainingTest, branches_independent)
{
    auto& solvers = ConstructSkeleton({0, 5, 7});

    ASSERT_TRUE(solvers[0].get().HasDependencies());
    ASSERT_FALSE(solvers[1].get().HasDependencies());
    ASSERT_FALSE(solvers[2].get().HasDependencies());
}

class Skeleton3DChainingTest : public SkeletonBaseTest
{
public: 
    std::vector<SolverRef>& ConstructSkeleton(std::vector<int> startIndices)
    {
        std::vector<BoneDesc> descriptors ={
            BoneDesc{Quaternion{  0.653,  0.271, 0.271, 0.653}, glm::sqrt(2), 0},            
            BoneDesc{Quaternion{  0.585, -0.811, 0.000, 0.000}, glm::sqrt(5), 1},
            BoneDesc{Quaternion{  0.447,  0.894, 0.000, 0.000}, glm::sqrt(5), 2},
            BoneDesc{Quaternion{ -0.447,  0.894, 0.000, 0.000}, glm::sqrt(5), 3},
            BoneDesc{Quaternion{  0.585,  0.811, 0.000, 0.000}, glm::sqrt(5), 4},
            BoneDesc{Quaternion{  0.811, -0.585, 0.000, 0.000}, glm::sqrt(2), 5},
            BoneDesc{Quaternion{  0.383,  0.924, 0.000, 0.000}, glm::sqrt(4), 6},
            BoneDesc{Quaternion{  0.851, -0.526, 0.000, 0.000}, glm::sqrt(1), 7},
            BoneDesc{Quaternion{  1.000,  0.000, 0.000, 0.000}, glm::sqrt(1), 8},
        };

        std::vector<std::vector<int>> structure = {
            std::vector<int>{0, 1, 2, 3, 4}, // spine
            {0, 1, 5, 6},                    // branch 1
            {0, 1, 2, 7, 8}                  // branch 2
        };

        m_solvers = CreateSolvers(descriptors, structure, startIndices, {m_target, m_target, m_target});
        return m_solvers;
    }

protected:
    TargetPosition m_target;
    std::vector<SolverRef> m_solvers;
};

TEST_F(Skeleton3DChainingTest, skeleton_structure)
{
    std::vector<Vector> bones {
        /*              0          1           2          3           4  */
        /*r*/ Vector{0, 0, 0}, {-1, 0, 1}, {-2, 0, -1}, {-3, 0, 1}, {-4, 0, -1},// {0, 0, 5},
        /*                   5          6     */
        /*branch 1*/{-2, 0, -1}, {-1, 0, -2},// {2, 0, 2},
        /*                   7          8     */
        /*branch 2*/{-3, 0, 1}, {-4, 0, 1},// {2, 4, 0}
    };

    TargetPosition target; 
    auto& solvers = ConstructSkeleton({0, 5, 7});
    const auto& skeletonBones = GetSkeleton().GetBones();
    for (size_t i = 0; i < bones.size(); ++i)
    {
        ASSERT_TRUE(TestHelpers::CompareVectors(bones[i], skeletonBones[i]->GetPosition(), 0.01)) << " Failed on " << i << "th iteration";
    }
}

class SkeletonForkTest : public SkeletonBaseTest
{
public: 
    std::vector<SolverRef>& ConstructSkeleton(std::vector<int> startIndices)
    {
        std::vector<std::vector<Vector>> bones ={
            /*                                           0          1          2          3          4          */
            /*root chain*/ std::vector<Vector>{Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}},
            /*                                           5          6            */ 
            /*branch    */                    {      {0, 4, 0}, {1, 4, 0}, {2, 4, 0}}
        };

        std::vector<BoneDesc> descriptors = LightIKTestBody::ConstructSkeleton(bones);
        std::vector<std::vector<int>> structure = {
            std::vector<int>{0, 1, 2, 3, 4}, // spine
            {0, 1, 2, 3, 5, 6}               // branch
        };

        m_solvers = CreateSolvers(descriptors, structure, startIndices, {m_target, m_target, m_target});
        return m_solvers;
    }

protected:
    TargetPosition m_target;
    std::vector<SolverRef> m_solvers;
};

TEST_F(SkeletonForkTest, fork_dependencies)
{
    auto& solvers = ConstructSkeleton({4, 6});

    ASSERT_FALSE(solvers[0].get().HasDependencies());
    ASSERT_FALSE(solvers[1].get().HasDependencies());
}

TEST_F(SkeletonForkTest, secondary_chain_length)
{
    auto& solvers = ConstructSkeleton({4, 6});

    ASSERT_EQ(1LLU, solvers[1].get().GetChainSize());
}

TEST_F(SkeletonForkTest, secondary_rootchain_length)
{
    auto& solvers = ConstructSkeleton({4, 6});

    ASSERT_EQ(2LLU, GetSkeleton().GetRootChain(solvers[1]).size());
}

};
