#include <memory>
#include <gtest/gtest.h>
#include "light_ik/light_ik.h"
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

TEST(LightIKTest, can_create_solver)
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

    testing::AssertionResult CompareVectors(const Vector& reference, const Vector& result)
    {
        if (isnan(result.x) || isnan(result.y) || isnan(result.z))
        {
            return testing::AssertionFailure() << "Result is invalid (" << result.x << ", " << result.y << ", " << result.z << ")";
        }
        else if (glm::abs(glm::abs(result.x) - glm::abs(reference.x)) > DELTA
            || glm::abs(glm::abs(result.y) - glm::abs(reference.y)) > DELTA
            || glm::abs(glm::abs(result.z) - glm::abs(reference.z)) > DELTA )
        {
            return testing::AssertionFailure() << "Result mismatch. Expected (" 
                << (float)reference.x << ", " << (float)reference.y << ", " << (float)reference.z 
                << ") VS (" 
                << (float)result.x << ", " << (float)result.y << ", " << (float)result.z << ")";
        }
        return testing::AssertionSuccess();
    }

    testing::AssertionResult ReachThePoint(const Vector& target, const Vector& result)
    {
        if (isnan(result.x) || isnan(result.y) || isnan(result.z))
        {
            return testing::AssertionFailure() << "Result is invalid (" << result.x << ", " << result.y << ", " << result.z << ")";
        }
        real angle = glm::angle(target, result);
        
        if (isnan(angle) || angle > DELTA)
        {
            return testing::AssertionFailure() << "Result mismatch. Expected (" 
                << target.x << ", " << target.y << ", " << target.z 
                << ") VS (" 
                << result.x << ", " << result.y << ", " << result.z << ")";
        }
        return testing::AssertionSuccess();
    }

protected:
    Solver& GetSolver() 
    {
        return *m_library;
    }
private:
    std::unique_ptr<Solver> m_library;
};

TEST_F(SolverBaseTests, can_add_bone)
{
    ASSERT_NO_THROW(GetSolver().AddBone({0,0,1}));
}

TEST_F(SolverBaseTests, chain_length_default_zero)
{
    ASSERT_EQ(0, GetSolver().GetChainSize());
}

TEST_F(SolverBaseTests, chain_lenght_bone_increments)
{
    GetSolver().AddBone({0,0,1});
    ASSERT_EQ(1, GetSolver().GetChainSize());
}

TEST_F(SolverBaseTests, length_of_added_bone)
{
    GetSolver().AddBone({0, 0, 2});
    size_t index = GetSolver().GetChainSize() - 1LLU;
    ASSERT_FLOAT_EQ(2.f, GetSolver().GetBone(index).GetLength());
}

TEST_F(SolverBaseTests, chain_root_default_position)
{
    Vector result = GetSolver().GetRootPosition();
    ASSERT_TRUE(CompareVectors({0,0,0}, result));
}

TEST_F(SolverBaseTests, chain_root_override)
{
    Vector target;
    ASSERT_NO_THROW(GetSolver().OverrideRootPosition(target));
}

TEST_F(SolverBaseTests, chain_root_position)
{
    Vector target{1.f, 23.f, -75.f};
    GetSolver().OverrideRootPosition(target);
    Vector result = GetSolver().GetRootPosition();
    ASSERT_TRUE(CompareVectors(target, result));
}

TEST_F(SolverBaseTests, chain_target_default_position)
{
    Vector result = GetSolver().GetTargetPosition();
    ASSERT_TRUE(CompareVectors({0,0,0}, result));
}

TEST_F(SolverBaseTests, chain_target_set)
{
    Vector target;
    ASSERT_NO_THROW(GetSolver().SetTargetPosition(target));
}

TEST_F(SolverBaseTests, chain_target_position)
{
    Vector target{1.f, 23.f, -75.f};
    GetSolver().SetTargetPosition(target);
    Vector result = GetSolver().GetTargetPosition();
    ASSERT_TRUE(CompareVectors(target, result));
}

TEST_F(SolverBaseTests, tip_position_empty)
{
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,0,0}, result));
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
    virtual void SetUp()
    {
        GetSolver().AddBone({0,0,1});
    }

    virtual void TearDown()
    {
    }

private:
};

TEST_F(BoneLookAtTest, tip_position_no_step)
{
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,0,1}, result));
}

TEST_F(BoneLookAtTest, can_iterate_back)
{
    ASSERT_NO_THROW(GetSolver().IterateBack());
}

TEST_F(BoneLookAtTest, can_iterate_front)
{
    ASSERT_NO_THROW(GetSolver().IterateBack());
}

TEST_F(BoneLookAtTest, tip_position_one_bone_step)
{
    GetSolver().SetTargetPosition({0.f, 0.f, 1.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,0,1}, result));
}

TEST_F(BoneLookAtTest, tip_position_one_bone_step_rotated)
{
    GetSolver().SetTargetPosition({0.f, 1.f, 0.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,1,0}, result));
}

TEST_F(BoneLookAtTest, tip_position_forward_direction)
{
    GetSolver().SetTargetPosition({0.f, 0.f, 1.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,0,1}, result));
}

TEST_F(BoneLookAtTest, tip_position_backward_direction)
{
    GetSolver().SetTargetPosition({0.f, 0.f, -1.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,0,-1}, result));
}

TEST_F(BoneLookAtTest, tip_position_zero_direction_no_move)
{
    GetSolver().SetTargetPosition({0.f, 0.f, 0.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,0,1}, result));
}

class BoneChainTest : public SolverBaseTests
{
protected:
    virtual void SetUp()
    {
        GetSolver().AddBone({0,0,1});
        GetSolver().AddBone({0,0,2});
    }

    virtual void TearDown()
    {
    }

private:
};

TEST_F(BoneChainTest, can_iterate_back)
{
    ASSERT_NO_THROW(GetSolver().IterateBack());
}

TEST_F(BoneChainTest, can_iterate_front)
{
    ASSERT_NO_THROW(GetSolver().IterateFront());
}

TEST_F(BoneChainTest, tip_position_no_step)
{
    Vector result = GetSolver().GetTipPosition();
    ASSERT_TRUE(CompareVectors({0,0,2}, result));
}

class CustomChainStructureTest : public SolverBaseTests
{
protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }

    testing::AssertionResult ProcessBoneChain(const std::vector<Vector>& bones, const Vector& root, const Vector& target, size_t steps = 1)
    {
        GetSolver().OverrideRootPosition(root);
        for (const auto& bone : bones)
        {
            GetSolver().AddBone(bone);
        }
        GetSolver().SetTargetPosition(target);

        for (size_t i = 0; i < steps; ++i)
        {
            GetSolver().IterateBack();
            GetSolver().IterateFront();
        }

        Vector result = GetSolver().GetTipPosition();
        return CompareVectors(target, result);
    }

private:
};

TEST_F(CustomChainStructureTest, double_bone)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{0, 0, 1}, {0, 0, 2}}, {0, 0, 0}, {0, 1, 1}));
}

TEST_F(CustomChainStructureTest, double_bone_3D)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{1, 1, 1}, {1, 2, 1}}, {0, 0, 0}, {0, 1, 1}));
}

TEST_F(CustomChainStructureTest, double_bone_rotated_tip)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{0, 0, 1}, {0, 1, 1}}, {0, 0, 0}, {0, 0, 1.5}));
}

TEST_F(CustomChainStructureTest, double_bone_rotated_root_2D)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{1, 0, 0}, {1, 1, 0}}, {0, 0, 0}, {1, 1, 0}));
}

TEST_F(CustomChainStructureTest, double_bone_rotated_root)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{1, 0, 0}, {1, 1, 0}}, {0, 0, 0}, {1, 0, 1}));
}

TEST_F(CustomChainStructureTest, double_bone_rotated_root3D)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{1, 1, 0}, {1, 0, 0}}, {0, 0, 0}, {1, 1, 1}));
}

TEST_F(CustomChainStructureTest, shifted_root)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{0, 1, 2}, {0, 1, 3}}, {0, 1, 0}, {0, 0, 2.5}));
}

TEST_F(CustomChainStructureTest, tri_bone)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{2, 0, 0}, {2, 2, 0}, {0, 2, 0}}, {0, 0, 0}, {0, 6.0, 0}, 10));
}

TEST_F(CustomChainStructureTest, multi_bone_chain)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, {0, 1, 0}, {4.0, 4.0, 4.0}, 1));
}

TEST_F(CustomChainStructureTest, tri_bone_chain)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{0, 0, 2}, {0, 2, 2}, {2, 2, 0}}, {0, 0, 0}, {1.0, 3.0, 1.0}, 1));
}

TEST_F(CustomChainStructureTest, angular_bone_2D)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{0, 1, 2}, {0, 2, 0}, {0, 3, 0}}, {0, 0, 0}, {4, 0, 0}));
}

TEST_F(CustomChainStructureTest, angular_bone)
{
    ASSERT_TRUE(ProcessBoneChain({Vector{0, 1, 2}, {0, 2, 0}, {0, 3, 0}}, {0, 0, 0}, {3, 3, 0}));
}

/*struct TestChain
{
    size_t numBones = 2;
    size_t iterrationsCount = 1;
    Vector targetPosition{0.f, 0.f, 0.f};
    Vector finalTipPosition{0.f, 0.f, 0.f};
    std::string comment{""};
};

class ChainParametricTest : public ::testing::TestWithParam<TestChain>
{
protected:
    virtual void SetUp()
    {
        m_library = std::make_unique<Solver>(); 
        for (size_t i = 0; i < GetParam().numBones; ++i)
        {
            GetSolver().AddBone({0.f, 0.f, (float)(i + 1)});
        }
        GetSolver().SetTargetPosition(GetParam().targetPosition);
    }

    virtual void TearDown()
    {
        m_library = nullptr;
    }

    Solver& GetSolver() {return *m_library;}
private:
    std::unique_ptr<Solver> m_library;
};

TEST_P(ChainParametricTest, chain_target)
{
    for (size_t i = 0; i < GetParam().iterrationsCount; ++i)
    {
        GetSolver().IterateBack();
        GetSolver().IterateFront();
    }

    Vector result = GetSolver().GetTipPosition();
    
    ASSERT_NEAR(GetParam().finalTipPosition.x, result.x, DELTA);
    ASSERT_NEAR(GetParam().finalTipPosition.y, result.y, DELTA);
    ASSERT_NEAR(GetParam().finalTipPosition.z, result.z, DELTA);
}

INSTANTIATE_TEST_SUITE_P(
    SimpleChainParametricTest,
    ChainParametricTest,
    ::testing::Values(
        TestChain{ 2, 1, {0.f, 2.f, 0.f},   {0.f, 2.f, 0.f},    "base rotation" },
        TestChain{ 2, 1, {0.f, 1.f, 1.f},   {0.f, 1.f, 1.f},    "Q1" },
        TestChain{ 2, 1, {0.f, 1.f, -1.f},  {0.f, 1.f, -1.f},   "Q2" },
        TestChain{ 2, 1, {0.f, -1.f, -1.f}, {0.f, -1.f, -1.f},  "Q3" },
        TestChain{ 2, 1, {0.f, -1.f, 1.f},  {0.f, -1.f, 1.f},   "Q4" },
        TestChain{ 2, 1, {0.f, 0.f, -2.f},  {0.f, 0.f, -2.f},   "collinear" },
        TestChain{ 2, 1, {0.f, 0.f, 1.f},   {0.f, 0.f, 1.f},    "halved" },
        TestChain{ 2, 1, {0.f, 0.f, 10.f},  {0.f, 0.f, 2.f},    "unreachable" },
        TestChain{ 2, 1, {0.f, 2.f, 2.f},   {0.f, 1.414213f, 1.414213f},    "unreachable" },
        TestChain{ 2, 1, {0.f, 2.f, -2.f},  {0.f, 1.414213f, -1.414213f},   "unreachable" },
        TestChain{ 2, 1, {0.f, 0.f, 0.f},   {0.f, 0.f, 0.f},     "loop" },
        TestChain{ 3, 1, {0.f, 2.f, 2.f},   {0.f, 2.f, 2.f},     "chain_3" },
        TestChain{ 4, 1, {1.f, 2.f, 1.5f},  {1.f, 2.f, 1.5f},    "chain_4" },
        TestChain{ 5, 1, {1.f, 0.f, 1.5f},  {1.f, 0.f, 1.5f},    "chain_5" }
    ),
    [](const ::testing::TestParamInfo<ChainParametricTest::ParamType>& info)
    {
        std::ostringstream str;
        str << "chain_target_" << info.param.comment << "_" << info.param.numBones
            << "_" << info.param.targetPosition.x
            << "_" << info.param.targetPosition.y
            << "_" << info.param.targetPosition.z;

        std::string formattedString = str.str();
        size_t pos = 0;
        while ((pos = formattedString.find(' ', 0L)) < formattedString.size())
        {
            formattedString[pos] = '_';
        }
        while ((pos = formattedString.find('-', 0L)) < formattedString.size())
        {
            formattedString[pos] = 'n';
        }
        while ((pos = formattedString.find('.', 0L)) < formattedString.size())
        {
            formattedString[pos] = 'd';
        }
        return formattedString;
    }
);
/**/

};
