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

TEST(SolverTest, can_create_solver)
{
    std::unique_ptr<Solver> library;
    Bone bone(2, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    Bone ref;
    TargetPosition target;
    ASSERT_NO_THROW(library = std::make_unique<Solver>(BoneSubchain{std::ref(bone)}, target, ref));
};

class SolverBaseTests : public ::testing::Test, public LightIKTestBody
{
public: 
    SolverBaseTests()
    { }

protected:
    SolverBase& GetSolver() 
    {
        return *m_solver;
    }
private:
    Solver* m_solver = nullptr;
};

TEST_F(SolverBaseTests, default_root)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0}
    }, -1, target);
    ASSERT_TRUE(TestHelpers::CompareVectors({0,0,0}, solver.GetRootPosition()));
}

TEST_F(SolverBaseTests, subchain_root)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::identity<Quaternion>(), 2.f, 0},
        BoneDesc{ glm::identity<Quaternion>(), 1.f, 1}
    }, 1, target);
    ASSERT_TRUE(TestHelpers::CompareVectors({0,2,0}, solver.GetRootPosition()));
}

TEST_F(SolverBaseTests, subchain_root_complex)
{
    TargetPosition target;
    SolverBase& solver = GetSkeleton().AddSolver({
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 0},
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 1.f, 1},
        BoneDesc{ glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}), 2.f, 2},
    }, 2, target);
    ASSERT_TRUE(TestHelpers::CompareVectors({0,-1,2}, solver.GetRootPosition()));
}

class SolverTargetingTests : public ::testing::Test, public LightIKTestBody
{
public: 
    SolverTargetingTests()
    { }

    void SetUp()
    {
        m_solver = &GetSkeleton().AddSolver({
            BoneDesc{ glm::identity<Quaternion>(), 2.f, 0}
        }, 1, m_target);
    }

protected:
    SolverBase& GetSolver() 
    {
        return *m_solver;
    }
    TargetPosition& GetTarget() 
    {
        return m_target;
    }
private:
    SolverBase* m_solver = nullptr;
    TargetPosition m_target;
};

TEST_F(SolverTargetingTests, can_set_target)
{
    Vector target;
    ASSERT_NO_THROW(GetTarget().SetPosition(target));
}

TEST_F(SolverTargetingTests, tip_position_empty)
{
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(TestHelpers::CompareVectors({0,0,0}, result));
}

TEST_F(SolverTargetingTests, can_iterate_back)
{
    ASSERT_NO_THROW(GetSolver().Execute());
}

class BoneLookAtTest : public ::testing::Test, public LightIKTestBody
{
protected:
    
    void SetupChain(const std::vector<Vector>& chain, int chainStartIndex, const Vector& target)
    {
        m_target.SetPosition(target);
        m_solver = &AddSolver(chain, chainStartIndex, m_target);
    }

    void Step(size_t iterations)
    {
        GetSkeleton().Update(iterations);
        GetSkeleton().FinalizeChains();
    }

    SolverBase& GetSolver() 
    {
        return *m_solver;
    }

private:
    SolverBase* m_solver = nullptr;
    TargetPosition m_target;
};

TEST_F(BoneLookAtTest, simple)
{
    Vector target{1, 0, 0};
    SetupChain({Vector{0, 1, 0}}, 0, target);
    Step(1);    

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, overlength)
{
    Vector target{1, 0, 0};
    SetupChain({Vector{0, 1, 1}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target * sqrt(2), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, distant)
{
    Vector target{10, 0, 0};
    SetupChain({Vector{0, 1, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(target), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, rooted)
{
    Vector target{1, 1, 0};
    SetupChain({Vector{0, 1, 0}, {0, 1, 1}}, 1, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, behind)
{
    Vector target{0, 0, -1};
    SetupChain({Vector{0, 0, 1}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, forward)
{
    Vector target{0, 1, 0};
    SetupChain({Vector{0, 1, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

class BoneChainTest : public BoneLookAtTest
{
};

TEST_F(BoneChainTest, the_same_position)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, the_same_position_oriented)
{
    Vector target{0, 2, 2};
    SetupChain({Vector{0, 1, 1}, {0, 2, 2}}, 0, target);
    Step(1);    

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, simple_rotation)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, simple_rotation_oriented)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{1, 1, 1}, {2, 2, 2}}, 0, target);
    Step(1);    

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, full_chain_rotation)
{
    Vector target{0, 0, 2};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, full_chain_orientation)
{
    Vector target{0, -1, -1};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, chain_3D)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{1, 1, 1}, {1, 2, 1}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{0, 0, 1}, {0, 0, 2}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_3D)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{1, 1, 1}, {1, 2, 1}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_rotated_tip)
{
    Vector target{0, 0, 1.5};
    SetupChain({Vector{0, 0, 1}, {0, 1, 1}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}


TEST_F(BoneChainTest, double_bone_rotated_root_2D)
{
    Vector target{1, 1, 0};
    SetupChain({Vector{1, 0, 0}, {1, 1, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_rotated_root)
{
    Vector target{1, 0, 1};
    SetupChain({Vector{1, 0, 0}, {1, 1, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_rotated_root3D)
{
    Vector target{1, 1, 1};
    SetupChain({Vector{1, 1, 0}, {1, 0, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, shifted_root)
{
    Vector target{0, 0, 2.5};
    SetupChain({Vector{0, 1, 0}, {0, 1, 2}, {0, 1, 3}}, 1, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, tri_bone)
{
    Vector target{0, 6.0, 0};
    SetupChain({Vector{2, 0, 0}, {2, 2, 0}, {0, 2, 0}}, 0, target);
    Step(10);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_chain)
{
    Vector target{2, 2, 0};
    SetupChain({Vector{0, 0, -2}, {0, 2, -2}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, multi_bone_chain)
{
    Vector target{1, 4, 4};
    SetupChain({Vector{0, 1, 0}, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, 1, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, tri_bone_chain)
{
    Vector target{1.0, 3.0, 1.0};
    SetupChain({Vector{0, 0, 2}, {0, 2, 2}, {2, 2, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, angular_bone_2D)
{
    Vector target{4, 0, 0};
    SetupChain({Vector{0, 1, 2}, {0, 2, 0}, {0, 3, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, angular_bone)
{
    Vector target{3, 3, 0};
    SetupChain({Vector{0, 1, 2}, {0, 2, 0}, {0, 3, 0}}, 0, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, multi_bone_reach)
{
    Vector target{4, 6, 4};
    Vector root{0, 1, 0};
    SetupChain({root, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, 1, target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareDirections(target - root, GetSolver().GetTipPosition() - root));
}

TEST_F(BoneChainTest, multi_bone_reach_multistep)
{
    Vector target{4, 6, 4};
    Vector root{0, 1, 0};
    SetupChain({root, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, 1, target);

    Step(10);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

};
