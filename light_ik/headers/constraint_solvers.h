#pragma once
#include "types.h"
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#include <string>

namespace LightIK
{
struct ConstraintSolver
{
    virtual Quaternion  CalculateRotation(const Quaternion& rotation) = 0;

    virtual ~ConstraintSolver() = default;
};

struct PassThrough final : public ConstraintSolver
{
    PassThrough(const Constraints&) {};
    virtual Quaternion  CalculateRotation(const Quaternion& rotation) { return rotation; };
};


/// @brief Swing Twist angles based solver, utilises idea of separate swing and twist rotation of the bone in its local coordinates
///         The approach is simpler and faster than Euler approaches but less controllable.
class SwingTwistSolver final : public ConstraintSolver
{
public:
    SwingTwistSolver(const Constraints& constraint);
    ~SwingTwistSolver() = default;

    virtual Quaternion  CalculateRotation(const Quaternion& rotation) override;

private:
    real        m_minXLimit     = 1.;
    real        m_maxXLimit     = 1.;
    real        m_minZLimit     = 1.;
    real        m_maxZLimit     = 1.;
    real        m_minTwist      = -1.;
    real        m_maxTwist      = 1.;
    real        m_centerX       = 0;
    real        m_centerZ       = 0;
    bool        m_unlockedX     = true;
    bool        m_unlockedZ     = true;
};


/// @brief Euler angles based solvers

struct EulerSolver : public ConstraintSolver
{
    EulerSolver(const Constraints& constraint);

    virtual Quaternion  CalculateRotation(const Quaternion& rotation) final;

    virtual Vector      ToTaitBriant(const Quaternion& q) = 0;
    virtual Quaternion  FromTaitBriant(const Vector& a)   = 0;

private:
    Constraints m_constraints;
};

class ConstraintSolverXZY final : public EulerSolver
{
public:
    ConstraintSolverXZY(const Constraints& constraint) : EulerSolver(constraint) {};
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

class ConstraintSolverZXY final : public EulerSolver
{
public:
    ConstraintSolverZXY(const Constraints& constraint) : EulerSolver(constraint) {};
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

class ConstraintSolverYXZ final : public EulerSolver
{
public:
    ConstraintSolverYXZ(const Constraints& constraint) : EulerSolver(constraint) {};
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

class ConstraintSolverYZX final : public EulerSolver
{
public:
    ConstraintSolverYZX(const Constraints& constraint) : EulerSolver(constraint) {};
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

class ConstraintSolverXYZ final : public EulerSolver
{
public:
    ConstraintSolverXYZ(const Constraints& constraint) : EulerSolver(constraint) {};
    virtual Vector      ToTaitBriant(const Quaternion& q) override;
    virtual Quaternion  FromTaitBriant(const Vector& a) override;
};

class SolversFactory
{
public: 
    static std::unique_ptr<ConstraintSolver> BuildSolver(ConstraintModes solverMode, const Constraints& constraint);
};

}
