#pragma once
#include "types.h"

#include <vector>
#include <string>
#include <functional>
#include <memory>

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

namespace LightIK
{

class SolverBase;

class Bone
{
public:
    Bone();
    Bone(real length, const Quaternion& orientation); 
    
    // Global orientation of the bone in the system coordinates assotiated with the root bone
    void SetGlobalOrientation(const Quaternion& orientation);
    const Quaternion& GetGlobalOrientation() const  {return m_globalOrientation;}

    // Local orientation of the bone in the sustem assotiated with the parent bone
    void SetRotation(const Quaternion& rotation);
    const Quaternion& GetRotation() const           {return m_rotation;}

    // Geometric data of the bone
    const real GetLength() const                    {return m_length.l;}
    const real GetLength2() const                   {return m_length.l2;}

    // Roatation constraints of the bone
    void SetConstraints(Constraints && newConstraints);
    const Constraints& GetConstraints() const       {return m_constraints;}

    // Apply constraints on local rotation, to update it and prevent the bone to overcome its limitations
    Quaternion ApplyConstraint(const Quaternion& rotation) const;

    // Global position of the bone in the system associated with the root bone
    void SetPosition(const Vector& position)        {m_position = position;}
    const Vector& GetPosition() const               {return m_position;}

    void SetOwner(SolverBase* owner)                {m_owner = owner;}
    SolverBase* GetOwner() const                    {return m_owner;}
private:
    Quaternion  m_globalOrientation;
    Quaternion  m_rotation;

    Length      m_length;

    Constraints m_constraints;

    // position of the bone joint
    Vector      m_position = Vector(0,0,0);

    // index of the bone, if index is negative the bone does not exists
    SolverBase* m_owner = nullptr;
};

using BonePtr       = std::unique_ptr<Bone>;
using BoneRef       = std::reference_wrapper<Bone>;
using BoneSubchain  = std::vector<BoneRef>;

}
