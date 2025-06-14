#include <iostream>
#include "light_ik/light_ik.h"
#include <godot_cpp/classes/sprite2d.hpp>

namespace godot
{

class LightIKPlugin : public Sprite2D
{
    GDCLASS(LightIKPlugin, Sprite2D)

private:
    double  m_timePassed = 0.0;
    Vector2 m_rotationCenter{0., 0.};
    double  m_radius = 10.;
    bool    m_animate = true;

protected:
    static void _bind_methods();

    void set_center(Vector2 center) {m_rotationCenter = center;}
    Vector2 get_center() const {return m_rotationCenter; }

    void set_radius(double radius) {m_radius = radius;}
    double get_radius() const {return m_radius; }

    void set_animate(double animate) {m_animate = animate;}
    double get_animate() const {return m_animate; }
public:
    LightIKPlugin();
    ~LightIKPlugin();

    void _ready() override;
    void _process(double delta) override;


};

}