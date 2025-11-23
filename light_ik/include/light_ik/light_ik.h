#pragma once
#include <../headers/types.h>
#include <../headers/helpers.h>
#include <memory>

namespace LightIK
{

class Solver;

class LightIK
{
public:
    LightIK();
    ~LightIK();

    void Reset();
    void SetRootPosition(const Vector& rootPosition);
    void AddBone(const Vector& boneEnd);
    void AddBone(const Vector& boneEnd, const Vector& axis, real angle);
    CoordinateSystem GetBoneLocal(size_t index) const;
    Vector GetBoneAxis(size_t index) const;
    float  GetBoneLength(size_t index) const;
    void SetTargetPosition(const Vector& targetPosition);

    Vector GetTargetPosition() const;
    Vector GetTipPosition() const;

    size_t UpdateChainPosition(size_t iterrations = 1);

    void GetBoneRotations(std::vector<Matrix>& rotations) const;
    std::vector<RotationParameters> GetRotationParameters(Vector initialDirection) const;
    std::vector<RotationParameters> GetRelativeRotationParameters(Vector initialDirection) const;
    std::vector<Quaternion>  GetRelativeRotations(Vector initialDirection) const;
    std::vector<Matrix> GetRelativeRotationMatrices(Vector initialDirection) const;

private:
    std::unique_ptr<Solver> m_solver;
};

}