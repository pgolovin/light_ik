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

class BoneFlexibilityTest : public ::testing::Test
{
protected:
    BoneFlexibilityTest()
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
    }

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

protected:
    Solver& GetSolver() 
    {
        return *m_library;
    }
private:
    std::unique_ptr<Solver> m_library;
};

TEST_F(BoneFlexibilityTest, set_constraint)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Constraints constraints;
    ASSERT_TRUE(GetSolver().SetConstraint(1, std::move(constraints)));
}

TEST_F(BoneFlexibilityTest, set_constraint_wrong_bone)
{
    Vector target{0, 2, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Constraints constraints;
    ASSERT_FALSE(GetSolver().SetConstraint(2, std::move(constraints)));
}

TEST_F(BoneFlexibilityTest, dummy_constraints)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Constraints constraints;
    GetSolver().SetConstraint(1, std::move(constraints));

    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_stiff)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Constraints constraints = { 0.5 };
    GetSolver().SetConstraint(1, std::move(constraints));

    Step();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_stiff_direction)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}}, target);
    Constraints constraints = { 0.5 };
    GetSolver().SetConstraint(1, std::move(constraints));

    Step();

    ASSERT_TRUE(TestHelpers::CompareDirections(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_fixed)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {1, 0, 0}}, target);
    Constraints constraints = { 0 };
    GetSolver().SetConstraint(1, std::move(constraints));

    Step();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, simple_fixed_direction)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {1, 0, 0}}, target);
    Constraints constraints = { 0 };
    GetSolver().SetConstraint(1, std::move(constraints));

    Step();

    ASSERT_TRUE(TestHelpers::CompareDirections(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneFlexibilityTest, fixed_preserve_angle)
{
    Vector target{0, 1.5, 0};
    SetupChain({Vector{0, 0, 0}, {0, 1, 0}, {1, 1, 0}}, target);
    Constraints constraints = { 0 };
    GetSolver().SetConstraint(1, std::move(constraints));

    Step();
    const auto& bones   = GetSolver().GetBones();
    Vector axis1        = bones.bones[0].GetGlobalOrientation() * Helpers::DefaultAxis();
    Vector axis2        = bones.bones[1].GetGlobalOrientation() * Helpers::DefaultAxis();
    ASSERT_NEAR(0, glm::dot(axis1, axis2), TestTolerance);
}

class BoneConstraintsTest : public BoneFlexibilityTest
{
public:
    void SetUp() override
    {
        Vector target{1, 0, 0};
        SetupChain({Vector{0, 0, 0}, {0, 1, 0}}, target);
    }
};

TEST_F(BoneConstraintsTest, no_rotation)
{
    auto& bone = GetSolver().GetBones().bones.front();
    Constraints constraints = { 0, 0, 0, 0, 0, 0, 0 };
    GetSolver().SetConstraint(1, std::move(constraints));
    Vector testVector = glm::rotateX(Helpers::DefaultAxis(), glm::pi<real>()/4);
}

TEST_F(BoneConstraintsTest, DISABLED_look_at_constraint)
{
    
    Constraints constraints = { 1, -glm::pi<real>()/2.f, 0, 0, 0, 0, glm::pi<real>()/2.f };
    // the joint is fullly flexible, but rotation limits blocks it from any rotation
    GetSolver().SetConstraint(0, std::move(constraints));

    Step();

    ASSERT_TRUE(TestHelpers::CompareVectors({0, 1, 0}, GetSolver().GetTipPosition()));
}


};
