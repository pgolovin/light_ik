#pragma once
#include <gtest/gtest.h>
#include "light_ik/light_ik.h"


namespace LightIK
{

static const real TestTolerance     = 1e-7;
    
class TestHelpers
{
public:
    static testing::AssertionResult CompareVectors(const Vector& reference, const Vector& result)
    {
        if (isnan(result.x) || isnan(result.y) || isnan(result.z))
        {
            return testing::AssertionFailure() << "Result is invalid (" << result.x << ", " << result.y << ", " << result.z << ")";
        }
        else if (glm::abs(glm::abs(result.x - reference.x)) > TestTolerance
            || glm::abs(glm::abs(result.y - reference.y)) > TestTolerance
            || glm::abs(glm::abs(result.z - reference.z)) > TestTolerance )
        {
            return testing::AssertionFailure() << "Result mismatch. Expected (" 
                << (float)reference.x << ", " << (float)reference.y << ", " << (float)reference.z 
                << ") VS (" 
                << (float)result.x << ", " << (float)result.y << ", " << (float)result.z << ")";
        }
        return testing::AssertionSuccess();
    }

    static testing::AssertionResult CompareDirections(const Vector& reference, const Vector& result)
    {
        return CompareVectors(glm::normalize(reference), glm::normalize(result));
    }
};

}
