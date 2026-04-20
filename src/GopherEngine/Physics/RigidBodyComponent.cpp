#include <GopherEngine/Physics/RigidBodyComponent.hpp>

#include <GopherEngine/Core/Node.hpp>
#include <stdexcept>

namespace GopherEngine
{
    RigidBodyComponent::RigidBodyComponent(float mass)
        : mass_(mass)
    {
    }

    RigidBodyComponent::~RigidBodyComponent()
    {
        // To be implemented: unregister the rigid body from the physics world
    }

    void RigidBodyComponent::initialize(Node& node)
    {
        // To be implemented: register the rigid body with the physics world
    }

    float RigidBodyComponent::get_mass() const
    {
        return mass_;
    }
}