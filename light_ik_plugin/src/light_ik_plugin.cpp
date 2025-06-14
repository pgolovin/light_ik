#include "light_ik_plugin.h"

#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
namespace godot
{
void LightIKPlugin::_bind_methods()
{
    UtilityFunctions::print("Binding methods");

    ClassDB::bind_method(D_METHOD("get_center"), &LightIKPlugin::get_center);
    ClassDB::bind_method(D_METHOD("set_center", "center"), &LightIKPlugin::set_center);

    ClassDB::bind_method(D_METHOD("get_radius"), &LightIKPlugin::get_radius);
    ClassDB::bind_method(D_METHOD("set_radius", "radius"), &LightIKPlugin::set_radius);

    ADD_GROUP("Position", "center_");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "center_center"), "set_center", "get_center");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "center_radius"), "set_radius", "get_radius");

    ClassDB::bind_method(D_METHOD("get_animate"), &LightIKPlugin::get_animate);
    ClassDB::bind_method(D_METHOD("set_animate", "animate"), &LightIKPlugin::set_animate);

    ADD_GROUP("Animation", "anim_");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "anim_animate", PROPERTY_HINT_NONE, "Editor only"), "set_animate", "get_animate");
}

LightIKPlugin::LightIKPlugin()
{
    
}

LightIKPlugin::~LightIKPlugin()
{

}

void LightIKPlugin::_ready()
{
    UtilityFunctions::print("registered");
    if (Engine::get_singleton()->is_editor_hint())
    {
        UtilityFunctions::print("IN EDITOR");
    }
    else
    {
        UtilityFunctions::print("IN RUNTIME");
    }
}

void LightIKPlugin::_process(double delta)
{
    m_timePassed += delta;
    Vector2 newPosition;
    if (m_animate || !Engine::get_singleton()->is_editor_hint())
    {
        newPosition = Vector2(m_rotationCenter.x + m_radius * glm::sin(m_timePassed), m_rotationCenter.y + m_radius * glm::cos(m_timePassed));
    }
    else 
    {
        newPosition = m_rotationCenter;
    }
    set_position(newPosition);
}

}