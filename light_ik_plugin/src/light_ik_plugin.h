#pragma once

#include "helpers.h"

#include "light_ik/light_ik.h"
#include "joint_constraints.h"

#include <godot_cpp/classes/skeleton_modifier3d.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/variant/node_path.hpp>

namespace godot
{

class LightIKPlugin : public SkeletonModifier3D
{
    GDCLASS(LightIKPlugin, SkeletonModifier3D)

    DEFINE_PROPERTY(double, simulate);
    DEFINE_PROPERTY(String, root_bone);
    DEFINE_PROPERTY(String, tip_bone);
    DEFINE_PROPERTY(NodePath, target);
    DEFINE_PROPERTY(TypedArray<JointConstraints>, constraints_array);

public:
    LightIKPlugin();
    ~LightIKPlugin();

    void _ready() override;
    void _process(double delta) override;
    void _validate_property(godot::PropertyInfo& info);

protected:
    static void _bind_methods();

private:
    void MakeConstraints();
    void ValidateRootBone(PropertyInfo& info);
    void ValidateTipBone(PropertyInfo& info);

    String                  m_rootBoneName;
    int32_t                 m_rootBone = -1;

    String                  m_tipBoneName;
    int32_t                 m_tipBone = -1;

    NodePath                m_targetPath;    
    TypedArray<JointConstraints> m_constraintsArray;

    bool                    m_simulate = true;
};

}