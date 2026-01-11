/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "helpers.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "glm/gtx/euler_angles.hpp"

#include <iostream>
#include <iomanip>

namespace LightIK
{
    Vector Helpers::Normal(const Vector& axis1, const Vector& axis2)
    {
#ifndef NDEBUG
        if (glm::length2(axis1) < EPSILON || glm::length2(axis2) < EPSILON )
        {
            return {0.f, 1.f, 0.f};
        }
#endif
        Vector result   = glm::cross(axis1, axis2);
        // if front is 0 length, try to cross it with arbitrary vector (Z axis)
        if (glm::length2(result) < EPSILON)
        {
            result      = glm::cross(axis1, {0.f, 0.f, 1.f});
        }
        // well it is possible that Z is aligned with the front , then choose another arbitraty vector (Y)
        if (glm::length2(result) < EPSILON)
        {
            result      = glm::cross(axis1, {0.f, 1.f, 0.f});
        }        
        // here it cannot be zero, coz Y and Z are ortogonal, so front must have angle at least with one of them
#ifndef NDEBUG      
        if (glm::length2(result) < EPSILON)
        {
            assert(false);
        }
#endif
        return glm::normalize(result);
    }

    RotationParameters Helpers::CalculateParameters(const Vector& from, const Vector& to)
    {
        // if from and to vectors are colinear, return zero rotation
        // without this fix error can appear in calculation of close to zero angles that actually should be zero
        if (glm::length2(glm::cross(from, to)) < EPSILON && glm::dot(from, to) > 0)
        {
            // vectors are looking in an one direction
            return {DefaultAxis(), 0};
        }
        // calculate the rotation axis between the from and to vectors
        Vector rotationAxis = Helpers::Normal(from, to);
        // calculate angle around calculated axis
        real rotationAngle  = glm::orientedAngle(from, to, rotationAxis);
        // form the quaternion to reflect the rotation
        return {rotationAxis, rotationAngle};
    }

    Quaternion Helpers::CalculateRotation(const Vector& from, const Vector& to)
    {
        RotationParameters params = CalculateParameters(from, to);
        return glm::angleAxis(params.angle, params.axis);
    }

    Vector Helpers::ToLocal(const CoordinateSystem& localSystem, const Vector& vector)
    {
        return Vector{
            glm::dot(vector, localSystem[(size_t)Axis::x]), 
            glm::dot(vector, localSystem[(size_t)Axis::y]), 
            glm::dot(vector, localSystem[(size_t)Axis::z])
        };
    }

    Matrix Helpers::CalculateTransferMatrix(const CoordinateSystem& base, const CoordinateSystem& target)
    {
        return target * glm::transpose(base);
    }

    void Helpers::Print(const std::string& prefix, const Vector& value)
    {
        std::cout << std::setprecision(7) << prefix << ": " 
            << std::fixed << value.x << ", " 
            << std::fixed << value.y << ", " 
            << std::fixed << value.z << std::endl;
    }

    void Helpers::Print(const std::string& prefix, const Quaternion& value)
    {
        std::cout << std::setprecision(7) << prefix << ": " 
            << std::fixed << value.x << ", " 
            << std::fixed << value.y << ", " 
            << std::fixed << value.z << ", "
            << std::fixed << value.w << std::endl;
    }

    void Helpers::Print(const std::string& prefix, const Matrix& value)
    {
        std::cout << prefix << ": " << std::endl;
        Print("line 1", value[0]);
        Print("line 2", value[1]);
        Print("line 3", value[2]);
    }

    // Calculate Tait-Bryan angles calculatation in non-standard seqence: XZY
    //
    // The reference GT, conversion to matrix and extraction of angles
    // auto testMatrix = glm::mat4_cast(q);
    // Vector ref;
    // glm::extractEulerAngleXZY(testMatrix, ref.x, ref.z, ref.y);

    Vector Helpers::ToEulerXZY(const Quaternion& q)
    { 
        Vector result = {0, 0, glm::asin(glm::clamp((real)(2) * (q.w * q.z - q.x * q.y), (real)-1, (real)1))};

        // calculate X and Y angles
        Vector2 params = {
            (real)2 * (q.w * q.x + q.y * q.z),
            q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z};
        if (!glm::all(glm::equal(params, Vector2(0,0), EPSILON)))
        {
            result.x = glm::atan2(params.x, params.y);
        }

        params.x = (real)2 * (q.w * q.y + q.x * q.z);
        params.y = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
        if (!glm::all(glm::equal(params, Vector2(0,0), EPSILON)))
        {
            result.y = glm::atan2(params.x, params.y);
        }

        return result;
    }

    // Calculate quaternion from Tait-Bryaint angles directly without applying heavy triple q multiplication
    // 
    // The referenced GT:
    // Quaternion x = glm::angleAxis(angles.x, Vector{1, 0, 0});
    // Quaternion y = glm::angleAxis(angles.y, Vector{0, 1, 0});
    // Quaternion z = glm::angleAxis(angles.z, Vector{0, 0, 1});
    // return ref = ((x * z) * y);

    Quaternion Helpers::FromEulerXZY(const Vector& angles)
    {
        const Vector s = glm::sin(angles * (real)0.5);
        const Vector c = glm::cos(angles * (real)0.5);
        // calculate multiplication of 3 quaternions for each euler angle in sequence YXZ
        // Q = Qx * Qz * Qy
        return Quaternion{
            (c.x * c.y * c.z) + (s.x * s.y * s.z),
            (s.x * c.y * c.z) - (c.x * s.y * s.z),
            (c.x * s.y * c.z) - (s.x * c.y * s.z),
            (c.x * c.y * s.z) + (s.x * s.y * c.z)
        };
    }

}
