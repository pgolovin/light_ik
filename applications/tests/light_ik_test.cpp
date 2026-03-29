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
    TargetPosition target({2,0,0});
    size_t chainIndex = library->CreateIKChain({
        BoneDesc{glm::identity<Quaternion>(), 1, 0},
        BoneDesc{glm::identity<Quaternion>(), 1, 1}}, 0, target);

    ASSERT_EQ(1, library->Update());
};

TEST(LightIKTest, no_movement_returns_false)
{
    std::unique_ptr<LightIK> library = std::make_unique<LightIK>(3);
    TargetPosition target({0,2,0});
    size_t chainIndex = library->CreateIKChain({
        BoneDesc{glm::identity<Quaternion>(), 1, 0},
        BoneDesc{glm::identity<Quaternion>(), 1, 1}}, 0, target);

    ASSERT_EQ(0, library->Update());
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
        BuildChain({GetRoot(), {0, 1, -2}, {0, 3, -2}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}}, 0);
    }

protected:

    void BuildChain(const std::vector<Vector>& chain, size_t index)
    {
        auto descriptors = ConstructDescriptors(chain);
        GetLibrary().CreateIKChain(descriptors, index, m_target);
    }

    Vector ReconstructBoneChain()
    {
        const auto& localRotations = GetLibrary().GetDeltaRotations();

        Vector tip {0, 0, 0};
        Quaternion rotation = glm::identity<Quaternion>();

        for (size_t i = 0; i < localRotations.size(); ++i)
        {
            rotation = rotation * (*localRotations[i]);  
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

    TargetPosition& GetTarget()
    {
        return m_target;
    }
    
private:
    std::unique_ptr<LightIK> m_library;
    TargetPosition m_target;
    
};

TEST_F(LightIKCoordinateTests, initial_quaternions)
{
    const auto& quaternions = GetLibrary().GetDeltaRotations();
    std::vector<Quaternion> refRotations{
        glm::identity<Quaternion>(),
        glm::angleAxis(-glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::angleAxis(glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::angleAxis(glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::angleAxis(-glm::pi<real>()/2.f, Vector{1,0,0}),
        glm::identity<Quaternion>(),
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
    GetTarget().SetPosition(target);

    GetLibrary().Update(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d)
{
    Vector target{1, 4, 4};
    GetTarget().SetPosition(target);
    GetLibrary().Update(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_large)
{
    Vector target{4, 4, 4};
    GetTarget().SetPosition(target);
    GetLibrary().Update(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_steps_count)
{
    Vector target{4, 4, 4};
    GetTarget().SetPosition(target);
    size_t steps = GetLibrary().Update(10);

    ASSERT_EQ(1, steps);
}

TEST_F(LightIKCoordinateTests, simulate_3d_distant)
{
    Vector target{4, 6, 4};
    GetTarget().SetPosition(target);
    GetLibrary().Update(1);

    Vector tip = ReconstructBoneChain();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_reach_distant)
{
    Vector target{4, 6, 4};
    GetTarget().SetPosition(target);
    GetLibrary().Update(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_reach_distant_steps)
{
    Vector target{4, 6, 4};
    GetTarget().SetPosition(target);
    size_t steps = GetLibrary().Update(10);

    ASSERT_LT(1, steps);
    ASSERT_GT(10, steps);
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable)
{
    Vector target{4, 7, 4};
    GetTarget().SetPosition(target);
    GetLibrary().Update(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_FALSE(TestHelpers::CompareVectors(target, tip));
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable_direction)
{
    Vector target{4, 7, 4};
    GetTarget().SetPosition(target);
    GetLibrary().Update(10);

    Vector tip = ReconstructBoneChain();

    ASSERT_TRUE(TestHelpers::CompareDirections(target - GetRoot(), tip - GetRoot()));
}

TEST_F(LightIKCoordinateTests, simulate_3d_unreachable_steps)
{
    Vector target{4, 7, 4};
    GetTarget().SetPosition(target);
    size_t steps = GetLibrary().Update(10);

    ASSERT_EQ(10, steps);
}

TEST_F(LightIKCoordinateTests, create_internal_target)
{
    ASSERT_NO_THROW(GetLibrary().CreateInternalTarget());
}

TEST_F(LightIKCoordinateTests, internal_target_position)
{
    TargetBone target = GetLibrary().CreateInternalTarget();
    target.AssignBone(3);
    ASSERT_TRUE(TestHelpers::CompareVectors({0, 3, -2}, target.GetPosition()));
}

class LightIKCoordinationTests : public ::testing::Test, public LightIKTestBody
{
public: 
    LightIKCoordinationTests()
        : m_library(std::make_unique<LightIK>(10))
        , m_boneTarget(m_library->CreateInternalTarget())
    {        
    }

    void ConstructSkeleton()
    {
        std::vector<std::vector<Vector>> bones = {
            /*                                           0          1          2          3          4          */
            /*root chain*/ std::vector<Vector>{Vector{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}},
            /*                                           5          6            */ 
            /*branch 1 */                     {      {0, 2, 0}, {1, 2, 0}, {2, 2, 0}},
            /*                                           7          8            */ 
            /*branch 2 */                     {      {0, 4, 0}, {1, 4, 0}, {2, 4, 0}}
        };

        std::vector<BoneDesc> descriptors = LightIKTestBody::ConstructSkeleton(bones);

        std::vector<int> rootStructure {0, 1, 2, 3, 4};
        std::vector<BoneDesc> rootDescriptors;
        for (int index : rootStructure)
        {
            rootDescriptors.emplace_back(descriptors[index]);
        }
        GetLibrary().CreateIKChain(rootDescriptors, 0, m_spineTarget);
        
        std::vector<int> branchStructure {0, 1, 5, 6};
        std::vector<BoneDesc> branchDescriptors;
        for (int index : branchStructure)
        {
            branchDescriptors.emplace_back(descriptors[index]);
        }
        GetLibrary().CreateIKChain(branchDescriptors, 5, m_boneTarget);

        std::vector<int> passiveChain {0, 1, 2, 3, 7, 8};
        std::vector<BoneDesc> passiveDescriptors;
        for (int index : passiveChain)
        {
            passiveDescriptors.emplace_back(descriptors[index]);
        }
        GetLibrary().CreateChain(passiveDescriptors);
        m_boneTarget.AssignBone(8);
    }

    void SetUp()
    {
        ConstructSkeleton();
    }
    
    LightIK& GetLibrary() 
    {
        return *m_library;
    }

    Target& GetBoneTarget()             {return m_boneTarget;}
    TargetPosition& GetSpineTarget()    {return m_spineTarget;}
protected:
    std::unique_ptr<LightIK> m_library;
    TargetPosition m_spineTarget;
    TargetBone      m_boneTarget;
};

TEST_F(LightIKCoordinationTests, bone_target)
{
    ASSERT_TRUE(TestHelpers::CompareVectors(GetLibrary().GetBonePosition(8), GetBoneTarget().GetPosition()));
}

TEST_F(LightIKCoordinationTests, movement)
{
    GetSpineTarget().SetPosition({0, 2, 2});
    GetSkeleton().Update(1);
    
    ASSERT_TRUE(TestHelpers::CompareVectors(GetLibrary().GetBonePosition(8), GetBoneTarget().GetPosition()));
}

};
