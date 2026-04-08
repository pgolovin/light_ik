#pragma once
#include "types.h"
#include "bone.h"

namespace LightIK
{

class Bone;

/// @brief Base class for all solvers, can be used to define custom solver
struct SolverBase
{
    virtual ~SolverBase() = default;
    
    virtual void   SetTipPosition(Vector& position) = 0;

    // TODO: maybe remove
    virtual Vector GetTipPosition() const = 0;
    virtual Vector GetRootPosition() const = 0;
    virtual const Vector& GetTargetPosition() const = 0;
    
    virtual void   SetDependencies(bool hasDependencies) = 0;
    virtual bool   HasDependencies() const = 0;

    virtual bool   TargetReached() const = 0;
    virtual void   Execute() = 0;

    virtual size_t GetChainSize() const = 0;
};

using SolverPtr = std::unique_ptr<SolverBase>;
using SolverRef = std::reference_wrapper<SolverBase>;

}
