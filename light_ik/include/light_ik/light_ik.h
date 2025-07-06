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

    void SetRootPosition(const Vector& rootPosition);
    void AddBone(const Vector& boneEnd);
    void SetTargetPosition(const Vector& targetPosition);

    bool UpdateChainPosition(size_t iterrations = 1);

    void GetBoneRotations(std::vector<Matrix>& rotations) const;

private:
    std::unique_ptr<Solver> m_solver;
};

}