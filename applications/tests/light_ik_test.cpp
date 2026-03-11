#include <memory>

#include "light_ik/light_ik.h"
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
 
TEST(LightIKTest, can_create_library)
{
    std::unique_ptr<LightIK> library;
    ASSERT_NO_THROW(library = std::make_unique<LightIK>(3));
};

TEST(LightIKTest, movement_returns_true)
{
    std::unique_ptr<LightIK> library = std::make_unique<LightIK>(3);
    size_t chainIndex = library->CreateIKChain({
        BoneDesc{glm::identity<Quaternion>(), 1, 0},
        BoneDesc{glm::identity<Quaternion>(), 1, 1}}, 0);

    library->SetTargetPosition(chainIndex, {2,0,0});

    ASSERT_EQ(1, library->UpdateChains());
};

TEST(LightIKTest, no_movement_returns_false)
{
    std::unique_ptr<LightIK> library = std::make_unique<LightIK>(3);
    size_t chainIndex = library->CreateIKChain({
        BoneDesc{glm::identity<Quaternion>(), 1, 0},
        BoneDesc{glm::identity<Quaternion>(), 1, 1}}, 0);

    library->SetTargetPosition(chainIndex, {0,2,0});
 
    ASSERT_EQ(0, library->UpdateChains());
};

class LightIKCoordinateTests : public ::testing::Test, public LightIKTestBody
{
public: 
    LightIKCoordinateTests()
    {
        m_library = std::make_unique<LightIK>(6); 
    }

    void SetUp() override
    {
        BuildChain({GetRoot(), {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, 1);
    }

protected:

    void BuildChain(const std::vector<Vector>& chain, size_t index)
    {
        auto descriptors = ConstructDescriptors(chain);
        GetLibrary().CreateIKChain(descriptors, index);
    }

    Vector ReconstructBoneChain()
    {
        const auto& localRotations = GetLibrary().GetDeltaRotations(0);

        Vector tip = GetLibrary().GetRootPosition(0);
        Quaternion rotation = glm::identity<Quaternion>();

        for (size_t i = 0; i < localRotations.size(); ++i)
        {
            rotation = rotation * (*localRotations[i]);  
            tip += rotation * (Helpers::DefaultAxis() * GetLibrary().GetBoneLength(i + 1));
        }

        return tip;
    }

    LightIK& GetLibrary() 
    {
        return *m_library;
    }

    Vector GetRoot()
    {
        return Vector{0, 1, 0};
    }
    
private:
    std::unique_ptr<LightIK> m_library;
    
};

TEST_F(LightIKCoordinateTests, initial_quaternions)
{
    const auto& quaternions = GetLibrary().GetDeltaRotations(0);
    std::vector<Quaternion> refRotations{
        glm::angleAxis(-glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::angleAxis(glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::angleAxis(glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::angleAxis(-glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::identity<Quaternion>()
    };
    ASSERT_TRUE(TestHelpers::CompareRotations(refRotations, quaternions));
}

TEST_F(LightIKCoordinateTests, reconstruct_static_chain)
{
    Vector target{0, 5, 0};
    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_planar)
{
    Vector target{0, 4, 4};
    GetLibrary().SetTargetPosition(0, target);

    GetLibrary().UpdateChains(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d)
{
    Vector target{1, 4, 4};
    GetLibrary().SetTargetPosition(0, target);
    GetLibrary().UpdateChains(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_large)
{
    Vector target{4, 4, 4};
    GetLibrary().SetTargetPosition(0, target);
    GetLibrary().UpdateChains(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_steps_count)
{
    Vector target{4, 4, 4};
    GetLibrary().SetTargetPosition(0, target);
    size_t steps = GetLibrary().UpdateChains(10);

    ASSERT_EQ(1, steps);
}

TEST_F(LightIKCoordinateTests, simulate_3d_distant)
{
    Vector target{4, 6, 4};
    GetLibrary().SetTargetPosition(0, target);
    GetLibrary().UpdateChains(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_reach_distant)
{
    Vector target{4, 6, 4};
    GetLibrary().SetTargetPosition(0, target);
    GetLibrary().UpdateChains(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_reach_distant_steps)
{
    Vector target{4, 6, 4};
    GetLibrary().SetTargetPosition(0, target);
    size_t steps = GetLibrary().UpdateChains(10);

    ASSERT_LT(1, steps);
    ASSERT_GT(10, steps);
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable)
{
    Vector target{4, 7, 4};
    GetLibrary().SetTargetPosition(0, target);
    GetLibrary().UpdateChains(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable_direction)
{
    Vector target{4, 7, 4};
    GetLibrary().SetTargetPosition(0, target);
    GetLibrary().UpdateChains(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareDirections(target - GetRoot(), tip - GetRoot()));
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable_steps)
{
    Vector target{4, 7, 4};
    GetLibrary().SetTargetPosition(0, target);
    size_t steps = GetLibrary().UpdateChains(10);

    ASSERT_EQ(10, steps);
}

};
