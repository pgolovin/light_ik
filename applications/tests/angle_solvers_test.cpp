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

class ConstraintSolverTest : public ::testing::Test
{
protected:
    ConstraintSolverTest()
    {
        
    }

    template <typename T> 
    void CreateConstraintsSolver()
    {
        m_constraintSolver = std::make_unique<T>();
    }

    Quaternion GetConversionResult(const Quaternion& ref)
    {
        auto angles = m_constraintSolver->ToTaitBriant(ref);
        return m_constraintSolver->FromTaitBriant(angles);
    }

    ConstraintSolver& GetConstraintsSolver() {return *m_constraintSolver;}
protected:
    std::unique_ptr<ConstraintSolver> m_constraintSolver = nullptr;
};

TEST_F(ConstraintSolverTest, solver_yzx_xz)
{
    CreateConstraintsSolver<ConstraintSolverYZX>();
    Quaternion rotation;
    rotation = glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}) * glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_yzx_zx)
{
    CreateConstraintsSolver<ConstraintSolverYZX>();
    Quaternion rotation;
    rotation = glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1}) * glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_yxz_xz)
{
    CreateConstraintsSolver<ConstraintSolverYXZ>();
    Quaternion rotation;
    rotation = glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}) * glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_yxz_zx)
{
    CreateConstraintsSolver<ConstraintSolverYXZ>();
    Quaternion rotation;
    rotation = glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1}) * glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_xzy_xz)
{
    CreateConstraintsSolver<ConstraintSolverXZY>();
    Quaternion rotation;
    rotation = glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}) * glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_xzy_zx)
{
    CreateConstraintsSolver<ConstraintSolverXZY>();
    Quaternion rotation;
    rotation = glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1}) * glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_zxy_xz)
{
    CreateConstraintsSolver<ConstraintSolverZXY>();
    Quaternion rotation;
    rotation = glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}) * glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_zxy_zx)
{
    CreateConstraintsSolver<ConstraintSolverZXY>();
    Quaternion rotation;
    rotation = glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1}) * glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
};

TEST_F(ConstraintSolverTest, solver_xyz_xz)
{
    CreateConstraintsSolver<ConstraintSolverXYZ>();
    Quaternion rotation;
    rotation = glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0}) * glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_FLOAT_EQ(1., glm::abs(glm::dot(result, rotation)));
};

TEST_F(ConstraintSolverTest, solver_xyz_zx)
{
    CreateConstraintsSolver<ConstraintSolverXYZ>();
    Quaternion rotation;
    rotation = glm::angleAxis(-glm::pi<real>()/4, Vector{0,0,1}) * glm::angleAxis(glm::pi<real>()/2, Vector{1,0,0});
    Quaternion result = GetConversionResult(rotation);

    Vector test = result * Vector{0,0,1};
    ASSERT_FLOAT_EQ(1., glm::abs(glm::dot(result, rotation)));
};

TEST_F(ConstraintSolverTest, complex_rotation_zxy)
{
    Quaternion rotation = glm::normalize(glm::angleAxis(glm::pi<real>()/2., Vector(1,0,0)) * glm::angleAxis(glm::pi<real>()/2., Vector(1,0,0)) * glm::angleAxis(glm::pi<real>()/4., Vector(0,0,1)));
    CreateConstraintsSolver<ConstraintSolverZXY>();
    Quaternion result = GetConversionResult(rotation);
    Vector test = rotation * Helpers::DefaultAxis();
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
}

TEST_F(ConstraintSolverTest, complex_rotation_xzy)
{
    Quaternion rotation = glm::normalize(glm::angleAxis(glm::pi<real>()/2., Vector(1,0,0)) * glm::angleAxis(glm::pi<real>()/2., Vector(1,0,0)) * glm::angleAxis(glm::pi<real>()/4., Vector(0,0,1)));
    CreateConstraintsSolver<ConstraintSolverXZY>();
    Quaternion result = GetConversionResult(rotation);
    Vector test = rotation * Helpers::DefaultAxis();
    ASSERT_TRUE(TestHelpers::CompareRotations(rotation, result));
}

TEST_F(ConstraintSolverTest, complex_rotation_yxz)
{
    Quaternion rotation = glm::normalize(glm::angleAxis(glm::pi<real>()/2., Vector(1,0,0)) * glm::angleAxis(glm::pi<real>()/2., Vector(1,0,0)) * glm::angleAxis(glm::pi<real>()/4., Vector(0,0,1)));
    CreateConstraintsSolver<ConstraintSolverYXZ>();
    Quaternion result = GetConversionResult(rotation);
    Vector test = rotation * Helpers::DefaultAxis();
    ASSERT_FLOAT_EQ(1., glm::abs(glm::dot(result, rotation)));
}

class BoneConstraintSolverTest : public ::testing::Test
{
protected:
    BoneConstraintSolverTest()
    {
        
    }

    Bone& CreateBone(const Quaternion& direction)
    {
        m_bone = std::make_unique<Bone>(1., direction);
        return *m_bone;
    }

    Bone& GetBone() const {return *m_bone;}
protected:
    std::unique_ptr<Bone> m_bone = nullptr;
};

TEST_F(BoneConstraintSolverTest, identity)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Quaternion rotation = bone.CalculateConstraintRotation(glm::identity<Quaternion>());
    ASSERT_TRUE(TestHelpers::CompareRotations(glm::identity<Quaternion>(), rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_within_limit)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Quaternion xAngle = glm::angleAxis( glm::quarter_pi<real>(), Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(xAngle);
    ASSERT_TRUE(TestHelpers::CompareRotations(xAngle, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_limited)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x           = -glm::quarter_pi<real>();
    c.maxAngles.x           = glm::quarter_pi<real>();
    Quaternion xRef         = glm::angleAxis( glm::quarter_pi<real>(), Vector(1,0,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis( glm::half_pi<real>(), Vector(1,0,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_fixed)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x = 0;
    c.maxAngles.x = 0;
    bone.SetConstraints(std::move(c));
    Quaternion xRef   = glm::angleAxis(0., Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis( glm::half_pi<real>(), Vector(1,0,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_asymmetric)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x = -glm::quarter_pi<real>();
    c.maxAngles.x = glm::half_pi<real>();
    bone.SetConstraints(std::move(c));
    Quaternion xRef   = glm::angleAxis(-glm::quarter_pi<real>(), Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis( -glm::half_pi<real>(), Vector(1,0,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_fixed_nonzero)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x = glm::quarter_pi<real>();
    c.maxAngles.x = glm::quarter_pi<real>();
    bone.SetConstraints(std::move(c));
    Quaternion xRef   = glm::angleAxis(glm::quarter_pi<real>(), Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis( glm::half_pi<real>(), Vector(1,0,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_non_centred_before)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x = glm::quarter_pi<real>();
    c.maxAngles.x = glm::half_pi<real>();
    bone.SetConstraints(std::move(c));
    Quaternion xRef   = glm::angleAxis(glm::quarter_pi<real>(), Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::identity<Quaternion>());
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_non_centred_inside)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x = glm::quarter_pi<real>();
    c.maxAngles.x = glm::half_pi<real>();
    bone.SetConstraints(std::move(c));
    Quaternion xRef   = glm::angleAxis(glm::pi<real>() / 3.f, Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(xRef);
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_non_centred_after)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x = glm::quarter_pi<real>();
    c.maxAngles.x = glm::half_pi<real>();
    bone.SetConstraints(std::move(c));
    Quaternion xRef   = glm::angleAxis(glm::half_pi<real>(), Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis(glm::pi<real>(), Vector(1,0,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_off_prime_angle)
{
    auto& bone = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x = -glm::half_pi<real>();
    c.maxAngles.x = 2 * glm::pi<real>() / 3.;
    bone.SetConstraints(std::move(c));
    Quaternion xRef   = glm::angleAxis(2 * glm::pi<real>() / 3, Vector(1,0,0));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis(5 * glm::pi<real>() / 6., Vector(1,0,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_off_prime_angle_inside)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x           = -glm::half_pi<real>();
    c.maxAngles.x           = 2 * glm::pi<real>() / 3.;
    Quaternion xRef         = glm::angleAxis(glm::half_pi<real>() + 0.01f, Vector(1,0,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(xRef);
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_zero_edge)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x           = 0;
    c.maxAngles.x           = glm::half_pi<real>();
    Quaternion xRef         = glm::identity<Quaternion>();

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis(-glm::half_pi<real>(), Vector(1,0,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, swing_x_center_defined)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.x           = glm::quarter_pi<real>();
    c.maxAngles.x           = glm::pi<real>();
    c.restAngles.x          = glm::pi<real>() / 2;
    Quaternion xRef         = glm::angleAxis(c.minAngles.x, Vector(1,0,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::identity<Quaternion>());
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, twist_unlimited)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Quaternion xRef         = glm::angleAxis(glm::pi<real>(), Vector(0,1,0));

    Quaternion rotation = bone.CalculateConstraintRotation(xRef);
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, twist_limited)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.y           = 0;
    c.maxAngles.y           = glm::quarter_pi<real>();
    Quaternion xRef         = glm::angleAxis(glm::quarter_pi<real>(), Vector(0,1,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis(glm::half_pi<real>(), Vector(0,1,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, twist_limited_inside)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.y           = 0;
    c.maxAngles.y           = glm::quarter_pi<real>();
    Quaternion xRef         = glm::angleAxis(glm::pi<real>() / 5., Vector(0,1,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(xRef);
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, twist_limited_left)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.y           = 0;
    c.maxAngles.y           = glm::quarter_pi<real>();
    Quaternion xRef         = glm::identity<Quaternion>();

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis(-glm::quarter_pi<real>(), Vector(0,1,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, twist_limited_negative_left)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.y           = -glm::quarter_pi<real>();
    c.maxAngles.y           = glm::quarter_pi<real>();
    Quaternion xRef         = glm::angleAxis(-glm::quarter_pi<real>(), Vector(0,1,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::angleAxis(-glm::half_pi<real>(), Vector(0,1,0)));
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, twist_positive_offset)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.y           = glm::quarter_pi<real>();
    c.maxAngles.y           = glm::half_pi<real>();
    Quaternion xRef         = glm::angleAxis(glm::quarter_pi<real>(), Vector(0,1,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::identity<Quaternion>());
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, twist_negative_offset)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles.y           = -glm::half_pi<real>();
    c.maxAngles.y           = -glm::quarter_pi<real>();
    Quaternion xRef         = glm::angleAxis(-glm::quarter_pi<real>(), Vector(0,1,0));

    bone.SetConstraints(std::move(c));
    Quaternion rotation = bone.CalculateConstraintRotation(glm::identity<Quaternion>());
    ASSERT_TRUE(TestHelpers::CompareRotations(xRef, rotation));
}

TEST_F(BoneConstraintSolverTest, complex_rotation_limited)
{
    auto& bone              = CreateBone(glm::identity<Quaternion>());
    Constraints c;
    c.minAngles             = Vector {glm::radians(0.),   -glm::radians(90.),  glm::radians(0.)};
    c.maxAngles             = Vector {glm::radians(120.),  glm::radians(90.),  glm::radians(90.)};
    bone.SetConstraints(std::move(c));

    Quaternion xRef         = glm::angleAxis(glm::radians(45.),    Vector(0,0,1)) * glm::angleAxis(glm::radians(80.), Vector(1,0,0));
    Vector before           = xRef * Helpers::DefaultAxis();
    Quaternion rotation = bone.CalculateConstraintRotation(xRef);
    Vector after            = rotation * Helpers::DefaultAxis();
    ASSERT_FLOAT_EQ(1., glm::dot(xRef, rotation));
}

};
