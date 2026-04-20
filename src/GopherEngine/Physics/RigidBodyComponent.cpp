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
        if (collider_id_ != InvalidColliderId)
            PhysicsWorld::get().unregister_rigid_body(collider_id_);
    }

    void RigidBodyComponent::initialize(Node& node)
    {
        if (mass_ <= 0.f)
            throw std::invalid_argument("RigidBodyComponent mass must be greater than zero");
        
        // Simulated rigid bodies write world-space transforms back into the node.
        // For this first teaching pass, only top-level scene nodes are supported.
        if (node.parent() == nullptr || node.parent()->parent() != nullptr)
            throw std::invalid_argument("RigidBodyComponent currently supports only top-level scene nodes");
        
        collider_id_ = PhysicsWorld::get().register_rigid_body(this, node);
    }

    float RigidBodyComponent::get_mass() const
    {
        return mass_;
    }
}