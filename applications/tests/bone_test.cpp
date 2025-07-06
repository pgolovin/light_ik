#include <memory>
#include <gtest/gtest.h>
#include "light_ik/light_ik.h"
#include "../../light_ik/headers/solver.h"
#include "../../light_ik/headers/bone.h"

#include <string>
#include <sstream>
#include <algorithm>

TEST(LightIKTest, can_create_solver)
{
    std::unique_ptr<LightIK::Solver> library;
    ASSERT_NO_THROW(library = std::make_unique<LightIK::Solver>());
};

class SolverBaseTests : public ::testing::Test
{
public: 
    SolverBaseTests()
    {
        m_library = std::make_unique<LightIK::Solver>(); 
    }
protected:
    virtual void SetUp()
    {    }

    virtual void TearDown()
    {    }

    LightIK::Solver& GetSolver() {return *m_library;}
private:
    std::unique_ptr<LightIK::Solver> m_library;
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

TEST_F(SolverBaseTests, angle_of_added_bone)
{
    GetSolver().AddBone({1, 1, 1});
    size_t index = GetSolver().GetChainSize() - 1LLU;
    ASSERT_FLOAT_EQ(0.f, GetSolver().GetBone(index).GetAngleX());
    ASSERT_FLOAT_EQ(0.f, GetSolver().GetBone(index).GetAngleY());
    ASSERT_FLOAT_EQ(0.f, GetSolver().GetBone(index).GetAngleZ());
}

TEST_F(SolverBaseTests, chain_root_default_position)
{
    LightIK::Vector result = GetSolver().GetRootPosition();
    ASSERT_FLOAT_EQ(0, result.x);
    ASSERT_FLOAT_EQ(0, result.y);
    ASSERT_FLOAT_EQ(0, result.z);
}

TEST_F(SolverBaseTests, chain_root_override)
{
    LightIK::Vector target;
    ASSERT_NO_THROW(GetSolver().OverrideRootPosition(target));
}

TEST_F(SolverBaseTests, chain_root_position)
{
    LightIK::Vector target{1.f, 23.f, -75.f};
    GetSolver().OverrideRootPosition(target);
    LightIK::Vector result = GetSolver().GetRootPosition();
    ASSERT_FLOAT_EQ(target.x, result.x);
    ASSERT_FLOAT_EQ(target.y, result.y);
    ASSERT_FLOAT_EQ(target.z, result.z);
}

TEST_F(SolverBaseTests, chain_target_default_position)
{
    LightIK::Vector result = GetSolver().GetTargetPosition();
    ASSERT_FLOAT_EQ(0, result.x);
    ASSERT_FLOAT_EQ(0, result.y);
    ASSERT_FLOAT_EQ(0, result.z);
}

TEST_F(SolverBaseTests, chain_target_set)
{
    LightIK::Vector target;
    ASSERT_NO_THROW(GetSolver().SetTargetPosition(target));
}

TEST_F(SolverBaseTests, chain_target_position)
{
    LightIK::Vector target{1.f, 23.f, -75.f};
    GetSolver().SetTargetPosition(target);
    LightIK::Vector result = GetSolver().GetTargetPosition();
    ASSERT_FLOAT_EQ(target.x, result.x);
    ASSERT_FLOAT_EQ(target.y, result.y);
    ASSERT_FLOAT_EQ(target.z, result.z);
}

TEST_F(SolverBaseTests, tip_position_empty)
{
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_FLOAT_EQ(0.f, result.x);
    ASSERT_FLOAT_EQ(0.f, result.y);
    ASSERT_FLOAT_EQ(0.f, result.z);
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
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_FLOAT_EQ(0.f, result.x);
    ASSERT_FLOAT_EQ(0.f, result.y);
    ASSERT_FLOAT_EQ(1.f, result.z);
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
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_FLOAT_EQ(0.f, result.x);
    ASSERT_FLOAT_EQ(0.f, result.y);
    ASSERT_FLOAT_EQ(1.f, result.z);
}

TEST_F(BoneLookAtTest, tip_position_one_bone_step_rotated)
{
    GetSolver().SetTargetPosition({0.f, 1.f, 0.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_NEAR(0.f, result.x, LightIK::DELTA);
    ASSERT_NEAR(1.f, result.y, LightIK::DELTA);
    ASSERT_NEAR(0.f, result.z, LightIK::DELTA);
}

TEST_F(BoneLookAtTest, tip_position_forward_direction)
{
    GetSolver().SetTargetPosition({0.f, 0.f, 1.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_NEAR(0.f, result.x, LightIK::DELTA);
    ASSERT_NEAR(0.f, result.y, LightIK::DELTA);
    ASSERT_NEAR(1.f, result.z, LightIK::DELTA);
}

TEST_F(BoneLookAtTest, tip_position_backward_direction)
{
    GetSolver().SetTargetPosition({0.f, 0.f, -1.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_NEAR(0.f, result.x, LightIK::DELTA);
    ASSERT_NEAR(0.f, result.y, LightIK::DELTA);
    ASSERT_NEAR(-1.f, result.z, LightIK::DELTA);
}

TEST_F(BoneLookAtTest, tip_position_zero_direction_no_move)
{
    GetSolver().SetTargetPosition({0.f, 0.f, 0.f});
    GetSolver().IterateBack();
    GetSolver().IterateFront();
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_NEAR(0.f, result.x, LightIK::DELTA);
    ASSERT_NEAR(0.f, result.y, LightIK::DELTA);
    ASSERT_NEAR(1.f, result.z, LightIK::DELTA);
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
    LightIK::Vector result = GetSolver().GetTipPosition();
    ASSERT_FLOAT_EQ(0.f, result.x);
    ASSERT_FLOAT_EQ(0.f, result.y);
    ASSERT_FLOAT_EQ(2.f, result.z);
}

TEST_F(BoneChainTest, debug_test)
{
    GetSolver().AddBone({0,0,3});
    GetSolver().AddBone({0,0,4});
    GetSolver().SetTargetPosition({0.f, 2.5f, 0.0f});

    GetSolver().IterateBack();
    GetSolver().IterateFront();

    LightIK::Vector result = GetSolver().GetTipPosition();

    ASSERT_NEAR(0.0f, result.x, LightIK::DELTA);
    ASSERT_NEAR(2.5f, result.y, LightIK::DELTA);
    ASSERT_NEAR(0.0f, result.z, LightIK::DELTA);
}

struct TestChain
{
    size_t numBones = 2;
    size_t iterrationsCount = 1;
    LightIK::Vector targetPosition{0.f, 0.f, 0.f};
    LightIK::Vector finalTipPosition{0.f, 0.f, 0.f};
    std::string comment{""};
};

class ChainParametricTest : public ::testing::TestWithParam<TestChain>
{
protected:
    virtual void SetUp()
    {
        m_library = std::make_unique<LightIK::Solver>(); 
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

    LightIK::Solver& GetSolver() {return *m_library;}
private:
    std::unique_ptr<LightIK::Solver> m_library;
};

TEST_P(ChainParametricTest, chain_target)
{
    for (size_t i = 0; i < GetParam().iterrationsCount; ++i)
    {
        GetSolver().IterateBack();
        GetSolver().IterateFront();
    }

    LightIK::Vector result = GetSolver().GetTipPosition();

    ASSERT_NEAR(GetParam().finalTipPosition.x, result.x, LightIK::DELTA);
    ASSERT_NEAR(GetParam().finalTipPosition.y, result.y, LightIK::DELTA);
    ASSERT_NEAR(GetParam().finalTipPosition.z, result.z, LightIK::DELTA);
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

