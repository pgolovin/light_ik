#include <memory>

#include "light_ik/light_ik.h"
#include "test_helpers.h"

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
    ASSERT_NO_THROW(library = std::make_unique<LightIK>());
};

TEST(LightIKTest, movement_returns_true)
{
    std::unique_ptr<LightIK> library = std::make_unique<LightIK>();
    library->SetRootPosition({0,0,0});
    library->AddBone(1, glm::identity<Quaternion>());
    library->AddBone(1, glm::identity<Quaternion>());
    library->SetTargetPosition({2,0,0});
 
    ASSERT_EQ(1, library->UpdateChainPosition());
};

TEST(LightIKTest, no_movement_returns_false)
{
    std::unique_ptr<LightIK> library = std::make_unique<LightIK>();
    library->SetRootPosition({0,0,0});
    library->AddBone(1, glm::identity<Quaternion>());
    library->AddBone(1, glm::identity<Quaternion>());
    library->SetTargetPosition({0,2,0});
 
    ASSERT_EQ(0, library->UpdateChainPosition());
};

class LightIKCoordinateTests : public ::testing::Test
{
public: 
    LightIKCoordinateTests()
    {
        m_library = std::make_unique<LightIK>(); 
    }

    void SetUp() override
    {
        BuildChain({GetRoot(), {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}});
    }

protected:

    void BuildChain(const std::vector<Vector>& chain)
    {
        m_library->SetRootPosition(chain.front());
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
        GetLibrary().CompleteChain();
    }

    Vector ReconstructBoneChain()
    {
        const std::vector<Quaternion>& localRotations = GetLibrary().GetDeltaRotations();

        Vector tip = GetLibrary().GetRootPosition();
        Quaternion rotation = glm::identity<Quaternion>();

        for (size_t i = 0; i < localRotations.size(); ++i)
        {
            // TODO: bug is here. first local rotation is applied to the bone, then it is rotated to a global position
            rotation = rotation * localRotations[i];  
            tip += rotation * (Helpers::DefaultAxis() * GetLibrary().GetBoneLength(i));
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
    const auto& quaternions = GetLibrary().GetDeltaRotations();
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
    GetLibrary().SetTargetPosition(target);

    GetLibrary().UpdateChainPosition(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d)
{
    Vector target{1, 4, 4};
    GetLibrary().SetTargetPosition(target);
    GetLibrary().UpdateChainPosition(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_large)
{
    Vector target{4, 4, 4};
    GetLibrary().SetTargetPosition(target);
    GetLibrary().UpdateChainPosition(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_steps_count)
{
    Vector target{4, 4, 4};
    GetLibrary().SetTargetPosition(target);
    size_t steps = GetLibrary().UpdateChainPosition(10);

    ASSERT_EQ(1, steps);
}

TEST_F(LightIKCoordinateTests, simulate_3d_distant)
{
    Vector target{4, 6, 4};
    GetLibrary().SetTargetPosition(target);
    GetLibrary().UpdateChainPosition(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_reach_distant)
{
    Vector target{4, 6, 4};
    GetLibrary().SetTargetPosition(target);
    GetLibrary().UpdateChainPosition(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_reach_distant_steps)
{
    Vector target{4, 6, 4};
    GetLibrary().SetTargetPosition(target);
    size_t steps = GetLibrary().UpdateChainPosition(10);

    ASSERT_LT(1, steps);
    ASSERT_GT(10, steps);
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable)
{
    Vector target{4, 7, 4};
    GetLibrary().SetTargetPosition(target);
    GetLibrary().UpdateChainPosition(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable_direction)
{
    Vector target{4, 7, 4};
    GetLibrary().SetTargetPosition(target);
    GetLibrary().UpdateChainPosition(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareDirections(target - GetRoot(), tip - GetRoot()));
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable_steps)
{
    Vector target{4, 7, 4};
    GetLibrary().SetTargetPosition(target);
    size_t steps = GetLibrary().UpdateChainPosition(10);

    ASSERT_EQ(10, steps);
}

};
