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
    ASSERT_EQ(0, GetSkeleton().GetChainsCount());
}

TEST_F(SkeletonBaseTest, new_chain)
{
    ASSERT_NO_THROW(AddSolver({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}}, 0));
}

TEST_F(SkeletonBaseTest, new_chain_added)
{
    AddSolver({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}}, 0);
    ASSERT_EQ(1LLU, GetSkeleton().GetChainsCount());
}

TEST_F(SkeletonBaseTest, new_chain_index)
{
    std::vector<Vector> chain = {Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}};
    Solver& solver = AddSolver(chain, 0);
}

TEST_F(SkeletonBaseTest, new_chain_size)
{
    std::vector<Vector> chain = {Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}};
    Solver& solver = AddSolver(chain, 0);
    ASSERT_EQ(chain.size(), GetSkeleton().GetRootChain(solver).size());
}

TEST_F(SkeletonBaseTest, new_chain_content)
{
    std::vector<Vector> chain = {Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}};
    Solver& solver = AddSolver(chain, 0);
    for (size_t i = 0; i < chain.size(); ++i)
    {
        TestHelpers::CompareVectors(chain[i], GetSkeleton().GetRootChain(solver)[i].get().GetPosition());
    }
}

TEST_F(SkeletonBaseTest, partial_ik_chain)
{
    Solver& solver = AddSolver({Vector{0,0,0}, Vector{0,1,0}, Vector{0,2,0}}, 1);
    auto& chain = GetSkeleton().GetRootChain(solver);
    ASSERT_FALSE(chain[0].get().IsInChain());
    ASSERT_TRUE(chain[1].get().IsInChain());
    ASSERT_TRUE(chain[2].get().IsInChain());
}

TEST_F(SkeletonBaseTest, add_second_chain)
{
    std::vector<Vector> chain {Vector{0,0,0}, {0,1,0}, {0,2,0}, {0,3,0}, {0,4,0}};
    AddSolver(chain, 0);

    std::vector<Vector> secondaryChain {Vector{1,0,0}, {1,1,0}, {1,2,0}};
    auto descriptors = ConstructDescriptors(secondaryChain);
    
    for (size_t i = 0; i < secondaryChain.size(); ++i)
    {
        descriptors[i].boneIndex = 5 + i;
    }
    GetSkeleton().AddSolver(descriptors, 0);
    ASSERT_EQ(2, GetSkeleton().GetChainsCount());
}


TEST_F(SkeletonBaseTest, one_bone_chain)
{
    ASSERT_NO_THROW(GetSkeleton().AddSolver({BoneDesc{glm::identity<Quaternion>(), 1.f, 1}}, 0));
}

TEST_F(SkeletonBaseTest, one_bone_chain_size)
{
    Solver& solver = GetSkeleton().AddSolver({BoneDesc{glm::identity<Quaternion>(), 1.f, 1}}, 0);
    ASSERT_EQ(1LLU, solver.GetChainSize());
}

TEST_F(SkeletonBaseTest, construct_chain)
{
    Solver& solver = GetSkeleton().AddSolver({BoneDesc{glm::identity<Quaternion>(), 1.f, 1}}, 0);
    ASSERT_NO_THROW(GetSkeleton().CompleteChain(solver));
}

TEST_F(SkeletonBaseTest, bone_positions)
{
    std::vector<Vector> chain{Vector{0, 1, 0}, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}};
    Solver& solver = AddSolver(chain, 0);

    GetSkeleton().CompleteChain(solver);
    const auto& rootChain = GetSkeleton().GetRootChain(solver);

    for(size_t i = 0; i < chain.size() - 1; ++i)
    {
        ASSERT_TRUE(TestHelpers::CompareVectors(chain[i], rootChain[i + 1].get().GetPosition())) << "Failed at " << i << "th";
    }
    ASSERT_TRUE(TestHelpers::CompareVectors(chain.back(), solver.GetTipPosition())) << "Failed at tip";
}

TEST_F(SkeletonBaseTest, tip_oriented_position)
{
    Solver& solver = GetSkeleton().AddSolver({BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 1}}, 0);
    GetSkeleton().CompleteChain(solver);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 0, 2), solver.GetTipPosition()));
}

TEST_F(SkeletonBaseTest, two_bone_chain)
{
    Solver& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::identity<Quaternion>(), 1.f, 1}
    }, 0);

    GetSkeleton().CompleteChain(solver);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 0, 3), solver.GetTipPosition()));
}

TEST_F(SkeletonBaseTest, two_bone_chain_oriented)
{
    Solver& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 1.f, 1}
    }, 0);

    GetSkeleton().CompleteChain(solver);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, -1, 2), solver.GetTipPosition()));
}

TEST_F(SkeletonBaseTest, two_bone_chain_oriented_3D)
{
    Solver& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{0,0,1}), 1.f, 1}
    }, 0);

    GetSkeleton().CompleteChain(solver);
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(-1, 0, 2), solver.GetTipPosition()));
}

class SkeletonChainingTest : public SkeletonBaseTest
{
public: 
    void SetUp()
    {
        std::vector<Vector> bones {
            /*                       0          1          2          3          4          5    */
            /*root chain*/ Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0},
            /*                       6          7    */ 
            /*branch 1 {0,2,0}*/ {1, 2, 0}, {2, 2, 0},
            /*                       8          9    */ 
            /*branch 2 {0,4,0}*/ {1, 4, 0}, {2, 4, 0}
        };
        m_descriptors = ConstructDescriptors(bones);
    }
    const std::vector<BoneDesc>& GetDescriptors() const { return m_descriptors; }

    std::vector<BoneDesc> GetSubchain(const std::vector<size_t>& indices)
    {
        std::vector<BoneDesc> subchain(indices.size());
        for (size_t i = 0; i < indices.size(); ++i)
        {
            subchain[i] = m_descriptors[indices[i]];
        }
        return subchain;
    }

protected:
    std::vector<BoneDesc> m_descriptors;
};

TEST_F(SkeletonChainingTest, add_branches)
{
    auto rootChain = GetSubchain({(size_t)0, 1, 2, 3, 4, 5});
    auto branch1 = GetSubchain({(size_t)0, 1, 2, 6, 7});
    auto branch2 = GetSubchain({(size_t)0, 1, 2, 3, 4, 8, 9});
    GetSkeleton().AddSolver(rootChain, 0);
    GetSkeleton().AddSolver(branch1, 6);
    GetSkeleton().AddSolver(branch2, 8);
    ASSERT_EQ(3, GetSkeleton().GetChainsCount());
}

TEST_F(SkeletonChainingTest, branch_truncation)
{
    auto rootChain = GetSubchain({(size_t)0, 1, 2, 3, 4, 5});
    auto branch1 = GetSubchain({(size_t)0, 1, 2, 6, 7});
    GetSkeleton().AddSolver(rootChain, 0);
    auto& branchSolver = GetSkeleton().AddSolver(branch1, 6);
    auto& branchChain = GetSkeleton().GetRootChain(branchSolver);
    ASSERT_EQ(3LLU, branchChain.size());
}

};
