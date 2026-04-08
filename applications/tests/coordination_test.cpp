#include <memory>
#include <gtest/gtest.h>

#include "test_helpers.h"
#include "test_body.h"

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

class CoordinationTest : public ::testing::Test, public LightIKTestBody
{
public: 
    CoordinationTest()
        : m_branch1(GetSkeleton())
        , m_branch2(GetSkeleton())
    {

    }

    std::vector<SolverRef> ConstructSkeleton(const std::vector<int>& startIndices, const std::vector<TargetRef>& targets)
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

        return CreateSolvers(descriptors, structure, startIndices, targets);
    }

    void SetUp()
    {
        m_solvers = ConstructSkeleton({0, 6, 8}, {m_root, m_branch1, m_branch2});

        m_branch1.AssignBone(8);
        m_branch2.AssignBone(6);
    }

    TargetPosition& GetRootTarget() {return m_root;}
    Target& GetBranch1Target() {return m_branch1;}
    Target& GetBranch2Target() {return m_branch2;}
private:
    TargetPosition m_root;
    TargetBone m_branch1;
    TargetBone m_branch2;
    std::vector<SolverRef> m_solvers;
};

TEST_F(CoordinationTest, bone_target)
{
    ASSERT_TRUE(TestHelpers::CompareVectors(GetSkeleton().GetBones().at(8)->GetPosition(), GetBranch1Target().GetPosition()));
}

TEST_F(CoordinationTest, movement_bottom_up)
{
    GetRootTarget().SetPosition({0, 2, 2});
    GetSkeleton().Update(1);
    
    ASSERT_TRUE(TestHelpers::CompareVectors(GetSkeleton().GetBones().at(8)->GetPosition(), GetBranch1Target().GetPosition()));
}

TEST_F(CoordinationTest, movement_top_down)
{
    GetRootTarget().SetPosition({0, 2, 2});
    GetSkeleton().Update(1);
    
    ASSERT_TRUE(TestHelpers::CompareVectors(GetSkeleton().GetBones().at(6)->GetPosition(), GetBranch2Target().GetPosition()));
}

class CoordinationWithPassiveChainTest : public ::testing::Test, public LightIKTestBody
{
public: 
    CoordinationWithPassiveChainTest() : m_boneTarget(GetSkeleton()) {}
    
    std::vector<SolverRef> ConstructSkeleton()
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
        std::vector<int> rootStructure {0, 1, 2, 3, 4};
        std::vector<int> branchStructure {0, 1, 5, 6};
        std::vector<int> passiveChain {0, 1, 2, 3, 7, 8};

        std::vector<SolverRef> solvers;
        solvers.emplace_back(CreateSolver(descriptors, rootStructure, 0, m_spineTarget));
        solvers.emplace_back(CreateSolver(descriptors, branchStructure, 5, m_boneTarget));
        solvers.emplace_back(*CreatePassiveChain(descriptors, passiveChain));

        return solvers;
    }

    void SetUp()
    {
        m_solvers = ConstructSkeleton();

        m_boneTarget.AssignBone(8);
    }

    Target& GetBoneTarget()  {return m_boneTarget;}
    TargetPosition& GetSpineTarget() {return m_spineTarget;}
protected:
    TargetPosition m_spineTarget;
    TargetBone      m_boneTarget;
    std::vector<SolverRef> m_solvers;
};

TEST_F(CoordinationWithPassiveChainTest, bone_target)
{
    ASSERT_TRUE(TestHelpers::CompareVectors(GetSkeleton().GetBones().at(8)->GetPosition(), GetBoneTarget().GetPosition()));
}

TEST_F(CoordinationWithPassiveChainTest, movement)
{
    GetSpineTarget().SetPosition({0, 2, 2});
    GetSkeleton().Update(1);
    
    ASSERT_TRUE(TestHelpers::CompareVectors(GetSkeleton().GetBones().at(8)->GetPosition(), GetBoneTarget().GetPosition()));
}

class CoordinationLinkChainTest : public ::testing::Test, public LightIKTestBody
{
public: 
    CoordinationLinkChainTest() : LightIKTestBody(11),  m_boneTarget(GetSkeleton()) {}
    
    std::vector<SolverRef> ConstructSkeleton()
    {
        std::vector<BoneDesc> descriptors ={
            BoneDesc{glm::identity<Quaternion>(), 1, 0},            
            BoneDesc{glm::identity<Quaternion>(), 1, 1},
            BoneDesc{glm::identity<Quaternion>(), 1, 2},
            BoneDesc{glm::identity<Quaternion>(), 1, 3},
            BoneDesc{glm::identity<Quaternion>(), 1, 4},
            /* 3 */
            BoneDesc{glm::identity<Quaternion>(), 0.5, 5},
            BoneDesc{glm::angleAxis(glm::pi<real>()/2, Vector(0,0,1)), 0.5, 6},
            BoneDesc{glm::angleAxis(glm::pi<real>()/2, Vector(0,0,1)), 0.5, 7},

            /* 0 */
            BoneDesc{glm::identity<Quaternion>(), 0.5, 8},
            BoneDesc{glm::angleAxis(glm::pi<real>()/2, Vector(0,0,1)), 0.5, 9},
            BoneDesc{glm::angleAxis(glm::pi<real>()/2, Vector(0,0,1)), 0.5, 10},
        };

        std::vector<int> rootStructure {0, 1, 2, 3, 4};
        std::vector<int> branchStructure {0, 1, 2, 3, 5, 6, 7};
        std::vector<int> passiveChain {0, 8, 9, 10};

        std::vector<SolverRef> solvers;
        solvers.emplace_back(CreateSolver(descriptors, rootStructure, 1, m_spineTarget));
        solvers.emplace_back(*CreatePassiveChain(descriptors, passiveChain));
        m_boneTarget.AssignBone(10);
        solvers.emplace_back(CreateSolver(descriptors, branchStructure, 7, m_boneTarget));

        return solvers;
    }

    void SetUp()
    {
        m_solvers = ConstructSkeleton();
    }

    Target& GetBoneTarget()  {return m_boneTarget;}
    TargetPosition& GetSpineTarget() {return m_spineTarget;}
protected:
    TargetPosition m_spineTarget;
    TargetBone      m_boneTarget;
    std::vector<SolverRef> m_solvers;
};

TEST_F(CoordinationLinkChainTest, movement)
{
    GetSpineTarget().SetPosition({4, 4, 0});
    GetSkeleton().Update(1);
    
    ASSERT_TRUE(TestHelpers::CompareVectors(GetSkeleton().GetBones().at(10)->GetPosition(), GetBoneTarget().GetPosition()));
}

TEST_F(CoordinationLinkChainTest, target_position)
{
    GetSpineTarget().SetPosition({0, 5, 0});

    for (size_t i = 0; i < 10; ++i)
    {
        GetSkeleton().Update(1);
    }
    
    auto& bone10 = GetSkeleton().GetBones().at(10);
    auto& bone7 = GetSkeleton().GetBones().at(7);
    Vector direction = bone7->GetGlobalOrientation() * Helpers::DefaultAxis();

    ASSERT_TRUE(TestHelpers::CompareDirections(bone10->GetPosition() - bone7->GetPosition(), direction));
}

};
