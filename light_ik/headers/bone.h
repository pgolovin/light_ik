#pragma once
#include "types.h"
#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

namespace LightIK
{

class Bone
{
public:
    Bone() = default;   
    Bone(real length, const Quaternion& orientation); 

    void SetGlobalOrientation(const Quaternion& orientation);
    const Quaternion& GetGlobalOrientation() const  {return m_globalOrientation;}

    void SetRotation(const Quaternion& rotation);
    const Quaternion& GetRotation() const           {return m_rotation;}

    const real GetLength() const                    {return m_length.l;}
    const real GetLength2() const                   {return m_length.l2;}

    void SetConstraints(Constraints && newConstraints);
    const Constraints& GetConstraints() const       {return m_constraints;}

    Quaternion ApplyConstraint(const Quaternion& rotation) const;

private:
    Length          m_length;
    Constraints     m_constraints;

    Quaternion      m_rotation;
    Quaternion      m_globalOrientation;
};

struct Pose
{
    std::vector<Bone>   bones;    
};

}
