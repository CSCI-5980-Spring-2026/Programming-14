#include <GopherEngine/Physics/ColliderComponent.hpp>

#include <GopherEngine/Core/Node.hpp>

namespace GopherEngine
{
    ColliderComponent::~ColliderComponent()
    {
        // The component owns its registration lifetime, so destroying the
        // component must also remove its Bullet representation.
        if (collider_id_ != InvalidColliderId)
            PhysicsWorld::get().unregister_collider(collider_id_);
    }

    void ColliderComponent::initialize(Node& node)
    {
        // Ask the derived class to register its concrete shape with PhysicsWorld.
        collider_id_ = register_with_physics(PhysicsWorld::get(), node);
    }

    ColliderId ColliderComponent::get_collider_id() const
    {
        return collider_id_;
    }

    bool ColliderComponent::is_colliding() const
    {
        return !overlapping_colliders_.empty();
    }

    const std::vector<ColliderId>& ColliderComponent::get_overlapping_colliders() const
    {
        return overlapping_colliders_;
    }
}
