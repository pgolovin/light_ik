/******************************************************************
  * Copyright: Pavel Golovinskiy 2025
*******************************************************************/

#include "helpers.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"
#include "glm/gtx/rotate_vector.hpp"

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

}
