# pragma once

#include <glm/vec3.hpp>

#include <variant>

namespace GopherEngine
{
    enum class ViewportMode
    {
        Fit,
        Crop,
        Stretch
    };

    struct Ray
    {
        glm::vec3 start{0.f};
        glm::vec3 end{0.f};
    };

    struct Sphere {
        float radius{0.5f}; 
    };

    struct Cylinder {
        float radius{0.5f}; 
        float height{1.f};
    };

    struct Cone {
        float radius{0.5f}; 
        float height{1.f}; 
    };

    struct Capsule {
        float radius{0.5f};
        float cylinder_height{1.f};
    };

    using ConvexShape = std::variant<Sphere, Cylinder, Cone, Capsule>;
}