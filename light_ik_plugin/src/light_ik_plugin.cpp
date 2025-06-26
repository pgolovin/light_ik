#include "light_ik_plugin.h"
#include "joint_constraints.h"

#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>

#include <stack>

namespace godot
{
void LightIKPlugin::_bind_methods()
{
    ADD_GROUP("IK Settings", "skeleton_");

    DECLARE_PROPERTY(LightIKPlugin, root_bone, (Variant::STRING), skeleton);
    DECLARE_PROPERTY(LightIKPlugin, tip_bone,  (Variant::STRING), skeleton);

    ClassDB::bind_method(D_METHOD("get_target"), &LightIKPlugin::get_target);
    ClassDB::bind_method(D_METHOD("set_target", "skeleton_target"), &LightIKPlugin::set_target);
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "skeleton_target", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"), "set_target", "get_target");

    ADD_GROUP("Constraints", "constraints_");

    ClassDB::bind_method(D_METHOD("get_constraints_array"), &LightIKPlugin::get_constraints_array);
    ClassDB::bind_method(D_METHOD("set_constraints_array", "constraints_array"), &LightIKPlugin::set_constraints_array);
    ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "constraints_constraints_array", PROPERTY_HINT_TYPE_STRING, 
            String::num(Variant::OBJECT) + "/" + String::num(PROPERTY_HINT_RESOURCE_TYPE) + ":JointConstraints"), "set_constraints_array", "get_constraints_array");

    ADD_GROUP("Runtime", "anim_");
    DECLARE_PROPERTY(LightIKPlugin, simulate,  (Variant::BOOL), anim);
}

void LightIKPlugin::set_simulate(const double& animate) 
{
    m_simulate = animate;
}
double LightIKPlugin::get_simulate() const 
{
    return m_simulate; 
}

void LightIKPlugin::set_root_bone(const String& root_bone_name) 
{
    m_rootBoneName = root_bone_name;
    if (get_skeleton())
    {
        m_rootBone = get_skeleton()->find_bone(root_bone_name);
        MakeConstraints();
    }
}
String LightIKPlugin::get_root_bone() const 
{
    return m_rootBoneName;
}

void LightIKPlugin::set_tip_bone(const String& tip_bone_name) 
{
    m_tipBoneName = tip_bone_name;
    if (get_skeleton())
    {
        m_tipBone = get_skeleton()->find_bone(tip_bone_name);
        MakeConstraints();
    }
}
String LightIKPlugin::get_tip_bone() const 
{
    return m_tipBoneName;
}

void LightIKPlugin::set_constraints_array(const TypedArray<JointConstraints>& array) 
{
    m_constraintsArray = array;
}
TypedArray<JointConstraints> LightIKPlugin::get_constraints_array() const 
{
    return m_constraintsArray; 
}

void LightIKPlugin::set_target(const NodePath& target_path) 
{
    m_targetPath = target_path;
}
NodePath LightIKPlugin::get_target() const 
{
    return m_targetPath;
}

LightIKPlugin::LightIKPlugin()
{
    
}

LightIKPlugin::~LightIKPlugin()
{

}

void LightIKPlugin::_ready()
{
    // on object initialization the skeleton doesn'o't exists, 
    // so parameters should be updated at the moment the object is constructed
    if (m_rootBoneName != "")
    {
        assert (get_skeleton());
        m_rootBone = get_skeleton()->find_bone(m_rootBoneName);
    }
    if (m_tipBoneName != "")
    {
        assert (get_skeleton());
        m_tipBone = get_skeleton()->find_bone(m_tipBoneName);
    }
}

void LightIKPlugin::_process(double delta)
{

}

void LightIKPlugin::_validate_property(godot::PropertyInfo& info)
{
    if (info.name == String("skeleton_root_bone"))
    {
        ValidateRootBone(info);
    }
    else if(info.name == String("skeleton_tip_bone"))
    {
        ValidateTipBone(info);
    }
}

void LightIKPlugin::ValidateRootBone(PropertyInfo& info)
{
    info.hint = PROPERTY_HINT_ENUM;
    if (!get_skeleton())
    {
        return;
    }
    if (-1 == m_tipBone)
    {
        // if tip is not selected allow to chose any bone for root
        info.hint_string = get_skeleton()->get_concatenated_bone_names();
        return;
    }

    // list all parent bones from current tip to skeleton root
    int32_t bone = m_tipBone;
    bool found = false;
    info.hint_string = "";

    while (bone >= 0)
    {
        // inverse addition to the list to keep the right order of the bones, from root to current
        info.hint_string = get_skeleton()->get_bone_name(bone) + "," + info.hint_string;
        found = found || (bone == m_rootBone);
        bone = get_skeleton()->get_bone_parent(bone);
    }

    // if root bone was not found in the branch of cuurent tip bone, it must be reseted
    if (!found)
    {
        m_rootBoneName = "";
        m_rootBone = -1;
    }
}

void LightIKPlugin::ValidateTipBone(PropertyInfo& info)
{
    if (!get_skeleton())
    {
        return;
    }

    info.hint = PROPERTY_HINT_ENUM;
    if (-1 == m_rootBone)
    {
        // if root is not selected allow to chose any bone for tip
        info.hint_string = get_skeleton()->get_concatenated_bone_names();
        return;
    }
    
    std::stack<int32_t> boneStack;
    // allow to chose any child bone from the selected root
    boneStack.emplace(m_rootBone);
    // create  the list of all child bones from current to all branch leaves
    // avoid recursion. create the stack of bones and process it.
    bool found = false;
    while (!boneStack.empty())
    {
        int32_t rootBone = boneStack.top();
        boneStack.pop();
        // concatenate names of child bones into the single list
        info.hint_string += get_skeleton()->get_bone_name(rootBone) + ",";
        found = found || rootBone == m_tipBone;

        // put all child bones to stack to process them later
        const auto& bones = get_skeleton()->get_bone_children(rootBone);
        for (int32_t bone : bones)
        {
            boneStack.emplace(bone);
        }
    }
    
    // if list doesn't contain current selected tip bone, drop the selection
    if (!found)
    {
        m_tipBoneName = "";
        m_tipBone = -1;
    }
}

void LightIKPlugin::MakeConstraints()
{
    if (-1 == m_tipBone || -1 == m_rootBone)
    {
        m_constraintsArray.clear();
    }
    else
    {
        int32_t bone = m_tipBone;
        int32_t bonesCount = 0;
        bool found = false;
        while (bone >= 0)
        {
            ++bonesCount;
            if (bone == m_rootBone)
            {
                found = true;
                break;
            }
            bone = get_skeleton()->get_bone_parent(bone);
        }
        if (!found)
        {
            UtilityFunctions::push_error("Bones ", m_rootBoneName, " and ", m_tipBoneName, " are not on the same branch.");
            return;
        }
        m_constraintsArray.resize(bonesCount);
    }
    notify_property_list_changed();
}

}