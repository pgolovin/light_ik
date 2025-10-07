#pragma once
#include <../headers/types.h>
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
    void SetTargetPosition(const Vector& targetPosition);

    Vector GetTargetPosition() const;
    Vector GetTipPosition() const;

    bool UpdateChainPosition(size_t iterrations = 1);

    void GetBoneRotations(std::vector<Matrix>& rotations) const;
    void GetRotationParameters(std::vector<RotationParameters>& rotations) const;
    void GetRelativeRotations(std::vector<RotationParameters>& rotations, Vector initialDirection) const;

private:
    std::unique_ptr<Solver> m_solver;
};

}