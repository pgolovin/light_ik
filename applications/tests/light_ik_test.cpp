#include <memory>

#include "light_ik/light_ik.h"
#include "test_helpers.h"

#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace LightIK
{
 
TEST(LightIKTest, can_create_library)
{
    std::unique_ptr<LightIK> library;
    ASSERT_NO_THROW(library = std::make_unique<LightIK>());
};

TEST(LightIKTest, movement_returns_true)
{
    std::unique_ptr<LightIK> library = std::make_unique<LightIK>();
    library->SetRootPosition({0,0,0});
   // library->AddBone({0,1,0});
   // library->AddBone({0,2,0});
    library->SetTargetPosition({2,0,0});
 
    //ASSERT_EQ(1, library->UpdateChainPosition());
};

TEST(LightIKTest, no_movement_returns_false)
{
    std::unique_ptr<LightIK> library = std::make_unique<LightIK>();
    library->SetRootPosition({0,0,0});
    // library->AddBone({0,1,0});
    // library->AddBone({0,2,0});
    library->SetTargetPosition({0,2,0});
 
    //ASSERT_EQ(0, library->UpdateChainPosition());
};
/*
class LightIKCoordinateTests : public ::testing::Test
{
public: 
    LightIKCoordinateTests()
    {
        m_library = std::make_unique<LightIK>(); 
    }

    void SetUp() override
    {
        DoStep({Vector{0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, {0, 1, 0}, {1.0, 4.0, 4.0}, 1);
        m_rotations     = GetLibrary().GetRelativeRotationMatrices({0,1,0});
        m_quaternions   = GetLibrary().GetRelativeRotations({0,1,0});
    }

protected:
    void DoStep(const std::vector<Vector>& bones, const Vector& root, const Vector& target, size_t steps = 1)
    {
        GetLibrary().SetRootPosition(root);
        for (const auto& bone : bones)
        {
            GetLibrary().AddBone(bone);
        }
        GetLibrary().SetTargetPosition(target);
        GetLibrary().UpdateChainPosition(steps);
    };

    LightIK& GetLibrary() 
    {
        return *m_library;
    }
    
    const std::vector<Matrix>& GetRotationMatrices() const { return m_rotations; }
    const std::vector<Quaternion>& GetRotationQuaternions() const { return m_quaternions; }
private:
    std::unique_ptr<LightIK> m_library;
    std::vector<Matrix> m_rotations;
    std::vector<Quaternion> m_quaternions;
};

TEST_F(LightIKCoordinateTests, matrix_dual_basis_rotated_synth)
{
    CoordinateSystem basis = glm::identity<CoordinateSystem>();
    CoordinateSystem target{Vector{-1, 0, 0}, Vector{0, 0, -1}, Vector{0, -1, 0}};
    Matrix rotation = Helpers::CalculateTransferMatrix(basis, target);

    ASSERT_TRUE(TestHelpers::CompareBasises(target, rotation * basis));
}

TEST_F(LightIKCoordinateTests, matrix_dual_basis_zero)
{
    CoordinateSystem basis = glm::identity<CoordinateSystem>();
    auto target = GetLibrary().GetBoneLocal(0);
    Matrix rotation = Helpers::CalculateTransferMatrix(basis, target);

    ASSERT_TRUE(TestHelpers::CompareBasises(target, rotation * basis));
}


TEST_F(LightIKCoordinateTests, matrix_dual_basis_rotated)
{
    auto basis      = GetLibrary().GetBoneLocal(0);
    auto target     = GetLibrary().GetBoneLocal(1);
    Matrix rotation = Helpers::CalculateTransferMatrix(basis, target);

    ASSERT_TRUE(TestHelpers::CompareBasises(target, rotation * basis));
}

TEST_F(LightIKCoordinateTests, matrix_axis_alignment)
{
    CoordinateSystem basis = glm::identity<CoordinateSystem>();
    auto target = GetLibrary().GetBoneLocal(0);

    ASSERT_TRUE(TestHelpers::CompareBasises(target, GetRotationMatrices()[0] * basis));
}


TEST_F(LightIKCoordinateTests, matrix_chain_alignment)
{
    CoordinateSystem basis = glm::identity<CoordinateSystem>();
    auto rotations = GetRotationMatrices();
    for (size_t i = 0; i < rotations.size(); ++i)
    {
        auto target = GetLibrary().GetBoneLocal(i);
        auto test = GetRotationMatrices()[i] * basis;

        ASSERT_TRUE(TestHelpers::CompareBasises(target, test)) << "failed on " << i <<"th element";
        basis = GetLibrary().GetBoneLocal(i);
    }
}

class LightIKBaseTests : public ::testing::Test
{
public: 
    LightIKBaseTests()
    {
        m_library = std::make_unique<LightIK>(); 
    }

    testing::AssertionResult Simulate(const std::vector<Vector>& bones, const Vector& root, const Vector& target, size_t steps = 1)
    {
        GetLibrary().SetRootPosition(root);
        for (const auto& bone : bones)
        {
            GetLibrary().AddBone(bone);
        }
        GetLibrary().SetTargetPosition(target);

        GetLibrary().UpdateChainPosition(steps);

        Vector result = root;
        Vector origResult = root;
        Vector initialBone{0,1,0};

        auto rotations = m_library->GetRelativeRotations(initialBone);
        // calculate axises directions in parent local coordinate systems
        Quaternion rotation = glm::identity<Quaternion>();
        for (size_t i = 0; i < bones.size(); ++i)
        {
            rotation    = rotation * rotations[i];
            result      += rotation * Helpers::DefaultAxis() * (real)m_library->GetBoneLength(i);
            std::cout << result.x << ", " << result.y << ", " << result.z << ";" << std::endl;
        }

        return TestHelpers::CompareVectors(target, result);
    }

protected:
    LightIK& GetLibrary() 
    {
        return *m_library;
    }
private:
    std::unique_ptr<LightIK> m_library;
};

TEST_F(LightIKBaseTests, multi_bone_reach)
{
    ASSERT_TRUE(Simulate({Vector{0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, {0, 1, 0}, {1.0, 4.0, 4.0}, 5));
}
*/   
};
