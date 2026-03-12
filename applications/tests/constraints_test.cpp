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

class BoneFlexibilityTest : public ::testing::Test, public LightIKTestBody
{
protected:
    BoneFlexibilityTest()
    {
        
    }

    void SetupChain(const std::vector<Vector>& chain, int index, const Vector& target)
    {
        m_solver = &AddSolver(chain, index);
        GetSolver().SetTargetPosition(target);
    }

    void Step(size_t iterations)
    {
        GetSkeleton().UpdateChains(iterations);
        GetSkeleton().FinalizeChains();
    }

protected:
    Solver& GetSolver() 
    {
        return *m_solver;
    }
private:
    Solver* m_solver;
};

TEST_F(BoneFlexibilityTest, set_constraint)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Constraints constraints;
    ASSERT_TRUE(GetSkeleton().SetConstraint(1, std::move(constraints)));
}

TEST_F(BoneFlexibilityTest, set_constraint_wrong_bone)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Constraints constraints;
    ASSERT_FALSE(GetSkeleton().SetConstraint(2, std::move(constraints)));
}

TEST_F(BoneFlexibilityTest, dummy_constraints)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Constraints constraints;
    GetSkeleton().SetConstraint(1, std::move(constraints));

    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_stiff)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Constraints constraints = { 0.5 };
    GetSkeleton().SetConstraint(1, std::move(constraints));

    Step(1);

    ASSERT_FALSE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_stiff_direction)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 1, 0}, {0, 2, 0}}, 0, target);
    Constraints constraints = { 0.5 };
    GetSkeleton().SetConstraint(1, std::move(constraints));

    Step(1);

    ASSERT_TRUE(TestHelpers::CompareDirections(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_fixed)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 1, 0}, {1, 0, 0}}, 0, target);
    Constraints constraints = { 0 };
    GetSkeleton().SetConstraint(1, std::move(constraints));

    Step(1);

    ASSERT_FALSE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_fixed_direction)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 1, 0}, {1, 0, 0}}, 0, target);
    Constraints constraints = { 0 };
    GetSkeleton().SetConstraint(1, std::move(constraints));

    Step(1);

    ASSERT_TRUE(TestHelpers::CompareDirections(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, fixed_preserve_angle)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 1, 0}, {1, 1, 0}}, 0, target);
    Constraints constraints = { 0 };
    GetSkeleton().SetConstraint(1, std::move(constraints));

    Step(1);
    const auto& bones   = GetSolver().GetChain();
    ASSERT_NE(0, bones.size());
    Vector axis1        = bones[0].get().GetGlobalOrientation() * Helpers::DefaultAxis();
    Vector axis2        = bones[1].get().GetGlobalOrientation() * Helpers::DefaultAxis();
    ASSERT_NEAR(0, glm::dot(axis1, axis2), TestTolerance);
}

class BoneLookAtConstraintsTest : public BoneFlexibilityTest
{
public:
    void SetUp() override
    {
        Vector target{1, 0, 0};
        SetupChain({Vector{0, 1, 0}}, 0, target);
    }
};

TEST_F(BoneLookAtConstraintsTest, free_rotation)
{
    //FIXME
    ASSERT_FALSE(GetSolver().GetChain().empty());

    Bone& bone = GetSolver().GetChain().front();
    Constraints constraints;
    GetSkeleton().SetConstraint(0, std::move(constraints));
    GetSolver().SetTargetPosition(Vector{1,0,0});
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors({1, 0, 0}, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, no_rotation)
{
    //FIXME
    ASSERT_FALSE(GetSolver().GetChain().empty());
    
    Bone& bone = GetSolver().GetChain().front();
    // the joint is fullly flexible, but rotation limits blocks it from any rotation
    Constraints constraints = { 1, Vector{0, 0, 0}, Vector{0, 0, 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    GetSolver().SetTargetPosition(Vector{1,0,0});
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors({0, 1, 0}, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, one_axis_allowed)
{
    Constraints constraints = { 1, Vector{-glm::pi<real>(), 0, 0}, Vector{glm::pi<real>(), 0, 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {0, 0, 1};
    GetSolver().SetTargetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, one_axis_blocked)
{
    Constraints constraints = { 1, Vector{-glm::pi<real>(), -glm::pi<real>(), 0}, Vector{glm::pi<real>(), glm::pi<real>(), 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {1, 0, 0};
    GetSolver().SetTargetPosition(target);
    Step(1);

    // TODO: think about
    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{0, 1, 0}), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, partially_blocked)
{
    Constraints constraints = { 1, Vector{-glm::pi<real>()/4, 0, 0}, Vector{glm::pi<real>()/4, 0, 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {0, 0, 1};
    GetSolver().SetTargetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{0, 1, 1}), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, sector_allowed)
{
    Constraints constraints = { 1, 
        Vector{-glm::pi<real>()/4, 0, -glm::pi<real>()/4}, 
        Vector{ glm::pi<real>()/4, 0,  glm::pi<real>()/4} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {1, 0, 1};
    GetSolver().SetTargetPosition(target);
    Step(1);

    // Constraints sequence XZY, so X gave the maximum angle, then Z, and the last one is Y. 
    // Thus we have the max X as sqrt(1/2), and the rest will equally divide the last distance, i think...
    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{sqrt(0.5), 0.5, 0.5}), GetSolver().GetTipPosition()));
}

class BoneRotationConstraintsTest : public BoneLookAtConstraintsTest
{
public:
    void SetUp() override
    {
        Vector target{1, 0, 0};
        SetupChain({Vector{0, 1, 0}, {0, 2, 0}, {0, 3, 0}}, 0, target);
    }
};

TEST_F(BoneRotationConstraintsTest, locked_bone)
{
    Constraints constraints = { 1, 
        Vector{0, 0, 0}, 
        Vector{0, 0, 0} };
    GetSkeleton().SetConstraint(1, std::move(constraints));
    Vector target {1.5, 0, 0};
    GetSolver().SetTargetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneRotationConstraintsTest, locked_bone_chain)
{
    Constraints constraints = { 1, 
        Vector{0, 0, 0}, 
        Vector{0, 0, 0} };
    GetSkeleton().SetConstraint(1, std::move(constraints));
    GetSkeleton().SetConstraint(2, std::move(constraints));
    Vector target {1.5, 0, 0};
    GetSolver().SetTargetPosition(target);
    Step(1);

    ASSERT_FALSE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneRotationConstraintsTest, locked_bone_chain_direction)
{
    Constraints constraints = { 1, 
        Vector{0, 0, 0}, 
        Vector{0, 0, 0} };
    GetSkeleton().SetConstraint(1, std::move(constraints));
    GetSkeleton().SetConstraint(2, std::move(constraints));
    Vector target {1.5, 0, 0};
    GetSolver().SetTargetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareDirections(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneRotationConstraintsTest, locked_bone_chain_target)
{
    Constraints constraints = { 1, 
        Vector{0, 0, 0}, 
        Vector{0, 0, 0} };
    GetSkeleton().SetConstraint(1, std::move(constraints));
    GetSkeleton().SetConstraint(2, std::move(constraints));
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(Vector{3, 0, 0}, GetSolver().GetTipPosition()));
}

class BoneChainConstraintsTest : public BoneRotationConstraintsTest
{
public:
    void SetUp() override
    {
        SetupChain({m_root, {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, 1, Vector{});
        Constraints constraints = { 1, 
            Vector{0, 0, 0}, 
            Vector{0, 0, 0} };
            
        //FIXME
        ASSERT_FALSE(GetSolver().GetChain().empty());

        for (size_t i = 0; i < GetSolver().GetChain().size() - 1; ++i)
        {
            // TODO: with skeleton is not working
            ASSERT_TRUE(GetSolver().SetConstraint(i + 1, std::move(constraints)));
        }
        
        GetSolver().SetTargetPosition(m_target);

    }
    const Vector& GetTarget() const {return m_target;}
    const Vector& GetRoot() const {return m_root;}
protected:
    const Vector m_target{2, 4, 5};
    const Vector m_root{0, 1, 0};
};

TEST_F(BoneChainConstraintsTest, strightening)
{
    Step(1);

    ASSERT_FALSE(TestHelpers::CompareVectors(GetTarget(), GetSolver().GetTipPosition()));
}

TEST_F(BoneChainConstraintsTest, actual_tip_position)
{
    Step(1);
    Vector direction = glm::normalize(GetTarget() - GetRoot());
    Vector tipPosition = direction * (real)8;
    ASSERT_TRUE(TestHelpers::CompareVectors(GetRoot() + tipPosition, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainConstraintsTest, actual_tip_position_multistep)
{
    Step(10);

    Vector direction = glm::normalize(GetTarget() - GetRoot());
    Vector tipPosition = direction * (real)8;
    ASSERT_TRUE(TestHelpers::CompareVectors(GetRoot() + tipPosition, GetSolver().GetTipPosition()));
}

};
