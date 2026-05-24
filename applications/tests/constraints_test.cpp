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
        m_solver = &AddSolver(chain, index, m_target);
        m_target.SetPosition(target);
    }

    void Step(size_t iterations)
    {
        GetSkeleton().Update(iterations);
        GetSkeleton().FinalizeChains();
    }

protected:
    Solver& GetSolver() 
    {
        return *static_cast<Solver*>(m_solver);
    }
    TargetPosition& GetTarget()
    {
        return m_target;
    }

private:
    SolverBase* m_solver;
    TargetPosition m_target;
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
    ASSERT_FALSE(GetSolver().GetChain().empty());

    Bone& bone = GetSolver().GetChain().front();
    Constraints constraints;
    GetSkeleton().SetConstraint(0, std::move(constraints));
    GetTarget().SetPosition(Vector{1,0,0});
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors({1, 0, 0}, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, no_rotation)
{
    ASSERT_FALSE(GetSolver().GetChain().empty());
    
    Bone& bone = GetSolver().GetChain().front();
    // the joint is fullly flexible, but rotation limits blocks it from any rotation
    Constraints constraints = { 1, Vector{0, 0, 0}, Vector{0, 0, 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    GetTarget().SetPosition(Vector{1,0,0});
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors({0, 1, 0}, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, one_axis_allowed)
{
    Constraints constraints = { 1, Vector{-glm::pi<real>(), 0, 0}, Vector{glm::pi<real>(), 0, 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {0, 0, 1};
    GetTarget().SetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(target, GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, one_axis_blocked)
{
    Constraints constraints = { 1, Vector{-glm::pi<real>(), -glm::pi<real>(), 0}, Vector{glm::pi<real>(), glm::pi<real>(), 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {1, 0, 0};
    GetTarget().SetPosition(target);
    Step(1);

    // TODO: think about
    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{0, 1, 0}), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, partially_blocked)
{
    Constraints constraints = { 1, Vector{-glm::pi<real>()/4, 0, 0}, Vector{glm::pi<real>()/4, 0, 0} };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {0, 0, 1};
    GetTarget().SetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{0, 1, 1}), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, sector_allowed_xzy)
{
    Constraints constraints = { 1, 
        Vector{-glm::pi<real>()/4, 0, -glm::pi<real>()/4}, 
        Vector{ glm::pi<real>()/4, 0,  glm::pi<real>()/4},
        ConstraintType::Local,
        ConstraintModes::XZY
    };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {1, 0, 1};
    GetTarget().SetPosition(target);
    Step(1);

    // Constraints sequence XZY, so X gave the maximum angle, then Z, and the last one is Y. 
    // Thus we have the max X as sqrt(1/2), and the rest will equally divide the last distance, i think...
    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{sqrt(0.5), 0.5, 0.5}), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, sector_allowed_zxy)
{
    Constraints constraints = { 1, 
        Vector{-glm::pi<real>()/4, 0, -glm::pi<real>()/4}, 
        Vector{ glm::pi<real>()/4, 0,  glm::pi<real>()/4},
        ConstraintType::Local,
        ConstraintModes::ZXY
    };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {1, 0, 1};
    GetTarget().SetPosition(target);
    Step(1);

    // Constraints sequence ZXY, so Z gave the maximum angle, then X, and the last one is Y. 
    // Thus we have the max Z as sqrt(1/2), and the rest will equally divide the last distance, i think...
    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{0.5, 0.5, sqrt(0.5)}), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, rotation_ccw)
{
    Constraints constraints = { 1 };
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {1, 0, 1};
    GetTarget().SetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{sqrt(0.5), 0, sqrt(0.5)}), GetSolver().GetTipPosition()));
}

TEST_F(BoneLookAtConstraintsTest, rotation_cw)
{
    Constraints constraints = { 1 };
    constraints.rotation = ConstraintRotation::CW;
    GetSkeleton().SetConstraint(0, std::move(constraints));
    Vector target {1, 0, 1};
    GetTarget().SetPosition(target);
    Step(1);

    ASSERT_TRUE(TestHelpers::CompareVectors(glm::normalize(Vector{sqrt(0.5), 0, sqrt(0.5)}), GetSolver().GetTipPosition()));
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
    GetTarget().SetPosition(target);
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
    GetTarget().SetPosition(target);
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
    GetTarget().SetPosition(target);
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
            
        ASSERT_FALSE(GetSolver().GetChain().empty());

        for (size_t i = 0; i < GetSolver().GetChain().size() - 1; ++i)
        {
            ASSERT_TRUE(GetSkeleton().SetConstraint(i + 2, std::move(constraints)));
        }
        
        GetTarget().SetPosition(m_target);

    }
    const Vector& GetTargetPosition() const {return m_target;}
    const Vector& GetRoot() const {return m_root;}
protected:
    const Vector m_target{2, 4, 5};
    const Vector m_root{0, 1, 0};
};

TEST_F(BoneChainConstraintsTest, strightening)
{
    Step(1);

    ASSERT_FALSE(TestHelpers::CompareVectors(GetTargetPosition(), GetSolver().GetTipPosition()));
}

TEST_F(BoneChainConstraintsTest, actual_tip_position)
{
    Step(1);
    Vector direction = glm::normalize(GetTargetPosition() - GetRoot());
    Vector tipPosition = direction * (real)8;
    ASSERT_TRUE(TestHelpers::CompareVectors(GetRoot() + tipPosition, GetSolver().GetTipPosition()));
}

TEST_F(BoneChainConstraintsTest, actual_tip_position_multistep)
{
    Step(10);

    Vector direction = glm::normalize(GetTargetPosition() - GetRoot());
    Vector tipPosition = direction * (real)8;
    ASSERT_TRUE(TestHelpers::CompareVectors(GetRoot() + tipPosition, GetSolver().GetTipPosition()));
}

class RootConstraintTest : public BoneChainConstraintsTest
{
public: 
    void SetUp() override
    {
        std::vector<BoneDesc> descriptors = {
            BoneDesc{glm::angleAxis(-glm::pi<real>()/4, Vector{1,0,0}),   glm::sqrt(2),  0},
            BoneDesc{glm::angleAxis(glm::pi<real>()/4, Vector{1,0,0}),  glm::sqrt(2),  1},
            BoneDesc{glm::identity<Quaternion>(),                        glm::sqrt(2),  2},
        };

        std::vector<int> rootStructure {0, 1, 2};
        m_solvers.emplace_back(CreateSolver(descriptors, rootStructure, 1, m_target));

        Constraints root  {1,   
            {Helpers::Grad2Rad(-90.), 0, 0}, 
            {Helpers::Grad2Rad( 90.), 0, 0}, 
            ConstraintType::Local,
            ConstraintModes::XZY
        };
        
        GetSkeleton().SetConstraint(1, std::move(root));
    }

    SolverBase& GetSolver()                 { return m_solvers.front();}
    TargetPosition& GetTarget()             { return m_target; }
protected:
    TargetPosition m_target;
    std::vector<SolverRef> m_solvers;
};

TEST_F(RootConstraintTest, root_limitation_reachable)
{
    SolverBase& solver = GetSolver();
    GetTarget().SetPosition(Vector(0, 1, 1));

    GetSkeleton().Update(1);
    GetSkeleton().FinalizeChains();

    ASSERT_TRUE(TestHelpers::CompareVectors(GetTarget().GetPosition(), solver.GetTipPosition()));
}

TEST_F(RootConstraintTest, root_limitation_unreachable)
{
    SolverBase& solver = GetSolver();
    GetTarget().SetPosition(Vector(0, 1, 2));

    GetSkeleton().Update(1);
    GetSkeleton().FinalizeChains();

    ASSERT_FALSE(TestHelpers::CompareVectors(GetTarget().GetPosition(), solver.GetTipPosition()));
}


class ComplexRotationsTest : public BoneRotationConstraintsTest
{
public: 
    void SetUp() override
    {
    }

    void ConstructSkeleton(std::vector<int> startIndices)
    {
        std::vector<BoneDesc> descriptors ={
            BoneDesc{Quaternion(  0.5,   0.5,   0.5,   -0.5),     1, 0},            
            BoneDesc{Quaternion(0.707, 0.707,   0.0,      0),   2.5, 1},
            BoneDesc{glm::identity<Quaternion>(),                 2, 2},
            BoneDesc{glm::identity<Quaternion>(),               0.5, 3},
            BoneDesc{Quaternion{0.707,     0,     0,  0.707},   0.7, 4},
            BoneDesc{glm::identity<Quaternion>(),               0.3, 5},
        };

        std::vector<int> rootStructure {0, 1, 2, 3, 4, 5};

        // TODO create global constraint and put it here
        m_solvers.emplace_back(CreateSolver(descriptors, rootStructure, startIndices[0], m_target));
    }

    void ApplyConstraints()
    {
        Constraints knee  {1, {0, 0, -170/180.*glm::pi<real>()}, {0, 0, 0}, ConstraintType::Local, ConstraintModes::ZXY, ConstraintRotation::CW};
        Constraints rotor {1, {0, -20/180.*glm::pi<real>(), 0}, {0, 20/180.*glm::pi<real>(), 0}, ConstraintType::Local, ConstraintModes::ZXY, ConstraintRotation::CCW};
        Constraints foot  {1, {0, 0, 25/180.*glm::pi<real>()}, {0, 0, 90/180.*glm::pi<real>()},  ConstraintType::Local, ConstraintModes::ZXY, ConstraintRotation::CCW};
        Constraints thumb {1, {0, 0, -20/180.*glm::pi<real>()}, {0, 0, 20/180.*glm::pi<real>()},  ConstraintType::Local, ConstraintModes::ZXY, ConstraintRotation::CCW};
        GetSkeleton().SetConstraint(2, std::move(knee));
        GetSkeleton().SetConstraint(3, std::move(rotor));
        GetSkeleton().SetConstraint(4, std::move(foot));
        GetSkeleton().SetConstraint(5, std::move(thumb));
    }

    std::vector<SolverRef>& GetSolvers()    { return m_solvers;}
    TargetPosition& GetTarget()             { return m_target; }
protected:
    TargetPosition m_target;
    std::vector<SolverRef> m_solvers;
};

TEST_F(ComplexRotationsTest, structure)
{
    ConstructSkeleton({0});
    auto& bones = GetSkeleton().GetBones();
    std::vector<Vector> positions = {
        Vector{0,0,0},
        {1,    0,     0},
        {1, -2.5,     0},
        {1, -4.5,     0},
        {1,   -5,     0},
        {1,   -5,   0.7},
    };
    for (size_t i = 0; i < positions.size(); ++i)
    {
        ASSERT_TRUE(TestHelpers::CompareVectors(positions[i], bones[i]->GetPosition(), 0.01)) << i << "th  position is wrong";
    }
}

TEST_F(ComplexRotationsTest, unlimited_rotations)
{
    ConstructSkeleton({0});
    GetTarget().SetPosition({1, -7, 0});

    for (size_t i = 0; i < 10; ++i)
    {
        GetSkeleton().Update(1);
    }

    ASSERT_TRUE(TestHelpers::CompareDirections({1, -7, 0}, GetSolvers().at(0).get().GetTipPosition()));    
}

TEST_F(ComplexRotationsTest, constrained_rotation)
{
    ConstructSkeleton({1});
    ApplyConstraints();
    GetTarget().SetPosition({1, -4, 0});

    for (size_t i = 0; i < 5; ++i)
    {
        GetSkeleton().Update(1);
    }

    ASSERT_TRUE(TestHelpers::CompareVectors({1, -4, -0}, GetSolvers().at(0).get().GetTipPosition()));    
}


};
