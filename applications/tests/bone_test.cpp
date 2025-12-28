#include <memory>
#include <gtest/gtest.h>

#include "test_helpers.h"
#include "../../light_ik/headers/solver.h"
#include "../../light_ik/headers/bone.h"
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

TEST(SolverTest, can_create_solver)
{
    std::unique_ptr<Solver> library;
    ASSERT_NO_THROW(library = std::make_unique<Solver>());
};

class SolverBaseTests : public ::testing::Test
{
public: 
    SolverBaseTests()
    {
        m_library = std::make_unique<Solver>(); 
    }

    void BuildChain(const std::vector<Vector>& chain)
    {
        m_library->OverrideRootPosition(chain.front());
        Vector direction{Helpers::DefaultAxis()};
        for (size_t i = 1; i < chain.size(); ++i)
        {
            Vector axis         = chain[i] - chain[i - 1];
            real length         = glm::length(axis);
            axis                = glm::normalize(axis);
            Vector rotationAxis = Helpers::Normal(direction, axis);
            real angle          = glm::orientedAngle(direction, axis, rotationAxis);
            m_library->AddBone(length, glm::angleAxis(angle, rotationAxis));
            std::swap(direction, axis);
        }
        m_library->CompleteChain();
    }

protected:
    Solver& GetSolver() 
    {
        return *m_library;
    }
private:
    std::unique_ptr<Solver> m_library;
};

TEST_F(SolverBaseTests, defult_joints)
{
    ASSERT_EQ(1, GetSolver().GetJoints().size());
}

TEST_F(SolverBaseTests, can_add_bone)
{
    ASSERT_NO_THROW(GetSolver().AddBone(1, glm::identity<Quaternion>()));
}

TEST_F(SolverBaseTests, new_bone_adds_joint)
{
    GetSolver().AddBone(1, glm::identity<Quaternion>());
    ASSERT_EQ(2, GetSolver().GetJoints().size());
}

TEST_F(SolverBaseTests, first_bone_joint_position)
{
    GetSolver().AddBone(1, glm::identity<Quaternion>());
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 1, 0), GetSolver().GetJoints().back()));
}

TEST_F(SolverBaseTests, first_bone_oriented_position)
{
    GetSolver().AddBone(2, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    GetSolver().CompleteChain();
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 0, 2), GetSolver().GetJoints().back()));
}

TEST_F(SolverBaseTests, second_bone_position)
{
    GetSolver().AddBone(2, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    GetSolver().AddBone(1, glm::identity<Quaternion>());
    GetSolver().CompleteChain();
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 0, 3), GetSolver().GetJoints().back()));
}

TEST_F(SolverBaseTests, second_bone_oriented_position)
{
    GetSolver().AddBone(2, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    GetSolver().AddBone(1, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    GetSolver().CompleteChain();
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, -1, 2), GetSolver().GetJoints().back()));
}

TEST_F(SolverBaseTests, second_bone_oriented_position_z)
{
    GetSolver().AddBone(2, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    GetSolver().AddBone(1, glm::angleAxis(glm::pi<real>()/2, Vector{0,0,1}));
    GetSolver().CompleteChain();
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(-1, 0, 2), GetSolver().GetJoints().back()));
}

TEST_F(SolverBaseTests, default_root)
{
    Vector result = GetSolver().GetRootPosition();
    ASSERT_TRUE(TestHelpers::CompareVectors({0,0,0}, result));
}

TEST_F(SolverBaseTests, reroot_empty_chain)
{
    GetSolver().OverrideRootPosition({0, 1, 0});
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(0, 1, 0), GetSolver().GetJoints().front()));
}

TEST_F(SolverBaseTests, reroot_chain_before)
{
    GetSolver().OverrideRootPosition({0, 1, 0});
    GetSolver().AddBone(2, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    GetSolver().AddBone(1, glm::angleAxis(glm::pi<real>()/2, Vector{0,0,1}));
    GetSolver().CompleteChain();
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(-1, 1, 2), GetSolver().GetJoints().back()));
}

TEST_F(SolverBaseTests, reroot_chain_after)
{
    GetSolver().AddBone(2, glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}));
    GetSolver().AddBone(1, glm::angleAxis(glm::pi<real>()/2, Vector{0,0,1}));
    GetSolver().OverrideRootPosition({0, 1, 0});
    GetSolver().CompleteChain();
    ASSERT_TRUE(TestHelpers::CompareVectors(Vector(-1, 1, 2), GetSolver().GetJoints().back()));
}

TEST_F(SolverBaseTests, flat_chain_joints)
{
    std::vector<Vector> chain{Vector{0, 1, 0}, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}};
    BuildChain(chain);
    auto& joints = GetSolver().GetJoints();
    for(size_t i = 0; i < joints.size(); ++i)
    {
        ASSERT_TRUE(TestHelpers::CompareVectors(chain[i], joints[i])) << "Failed at " << i << "th";
    }
}

TEST_F(SolverBaseTests, chain_target_default_position)
{
    Vector result = GetSolver().GetTargetPosition();
    ASSERT_TRUE(TestHelpers::CompareVectors({0,0,0}, result));
}

TEST_F(SolverBaseTests, can_set_target)
{
    Vector target;
    ASSERT_NO_THROW(GetSolver().SetTargetPosition(target));
}

TEST_F(SolverBaseTests, chain_target_position)
{
    Vector target{1.f, 23.f, -75.f};
    GetSolver().SetTargetPosition(target);
    Vector result = GetSolver().GetTargetPosition();
    ASSERT_TRUE(TestHelpers::CompareVectors(target, result));
}

TEST_F(SolverBaseTests, tip_position_empty)
{
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(TestHelpers::CompareVectors({0,0,0}, result));
}

TEST_F(SolverBaseTests, can_iterate_back)
{
    ASSERT_NO_THROW(GetSolver().IterateBack());
}

TEST_F(SolverBaseTests, can_iterate_front)
{
    ASSERT_NO_THROW(GetSolver().IterateBack());
}


class BoneLookAtTest : public SolverBaseTests
{
protected:
    
    void SetupChain(const std::vector<Vector>& chain, const Vector& target)
    {
        BuildChain(chain);
        GetSolver().SetTargetPosition(target);
        GetSolver().CompleteChain();
    }

    void Step()
    {
        GetSolver().IterateBack();
        GetSolver().IterateFront();
    }
private:
};

TEST_F(BoneLookAtTest, simple)
{
    Vector target{1, 0, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}}, target);
    Step();    

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, overlength)
{
    Vector target{1, 0, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 1}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target * sqrt(2), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, distant)
{
    Vector target{10, 0, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(target), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, rooted)
{
    Vector target{1, 1, 0};
    SetupChain({Vector{0, 1, 0}, {0, 1, 1}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, behind)
{
    Vector target{0, 0, -1};
    SetupChain({Vector{0, 0, 0}, {0, 0, 1}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtTest, forward)
{
    Vector target{0, 1, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

class BoneChainTest : public BoneLookAtTest
{
};

TEST_F(BoneChainTest, the_same_position)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, the_same_position_oriented)
{
    Vector target{0, 2, 2};
    SetupChain({Vector{0, 0, 0}, {0, 1, 1}, {0, 2, 2}}, target);
    Step();    

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, simple_rotation)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, simple_rotation_oriented)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{0, 0, 0}, {1, 1, 1}, {2, 2, 2}}, target);
    Step();    

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, full_chain_rotation)
{
    Vector target{0, 0, 2};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, full_chain_orientation)
{
    Vector target{0, -1, -1};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, chain_3D)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{0, 0, 0}, {1, 1, 1}, {1, 2, 1}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{0, 0, 0}, {0, 0, 1}, {0, 0, 2}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_3D)
{
    Vector target{0, 1, 1};
    SetupChain({Vector{0, 0, 0}, {1, 1, 1}, {1, 2, 1}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_rotated_tip)
{
    Vector target{0, 0, 1.5};
    SetupChain({Vector{0, 0, 0}, {0, 0, 1}, {0, 1, 1}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}


TEST_F(BoneChainTest, double_bone_rotated_root_2D)
{
    Vector target{1, 1, 0};
    SetupChain({Vector{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_rotated_root)
{
    Vector target{1, 0, 1};
    SetupChain({Vector{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_rotated_root3D)
{
    Vector target{1, 1, 1};
    SetupChain({Vector{0, 0, 0}, {1, 1, 0}, {1, 0, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, shifted_root)
{
    Vector target{0, 0, 2.5};
    SetupChain({Vector{0, 1, 0}, {0, 1, 2}, {0, 1, 3}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, tri_bone)
{
    Vector target{0, 6.0, 0};
    SetupChain({Vector{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}}, target);
    for (size_t i = 0; i < 10; ++i)
    {
        Step();
    }

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, double_bone_chain)
{
    Vector target{2, 2, 0};
    SetupChain({Vector{0, 0, 0}, {0, 0, -2}, {0, 2, -2}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, multi_bone_chain)
{
    Vector target{1, 4, 4};
    SetupChain({Vector{0, 1, 0}, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, tri_bone_chain)
{
    Vector target{1.0, 3.0, 1.0};
    SetupChain({Vector{0, 0, 0}, {0, 0, 2}, {0, 2, 2}, {2, 2, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, angular_bone_2D)
{
    Vector target{4, 0, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 2}, {0, 2, 0}, {0, 3, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, angular_bone)
{
    Vector target{3, 3, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 2}, {0, 2, 0}, {0, 3, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainTest, multi_bone_reach)
{
    Vector target{4, 6, 4};
    Vector root{0, 1, 0};
    SetupChain({root, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, target);
    Step();

    ASSERT_TRUE(TestHelpers::CompareDirections(target - root, GetSolver().GetTipPosition() - root));
}

TEST_F(BoneChainTest, multi_bone_reach_multistep)
{
    Vector target{4, 6, 4};
    Vector root{0, 1, 0};
    SetupChain({root, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, target);

    for (size_t i = 0; i < 10; ++i)
    {
        Step();
    }

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

};
