#pragma once

#include <GopherEngine/Core/Service.hpp>
#include <GopherEngine/Core/Types.hpp>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

// Forward declarations for Bullet types so the header does not need to include Bullet headers.
class btBroadphaseInterface;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btCollisionObject;
class btCollisionShape;
class btConstraintSolver;
class btDiscreteDynamicsWorld;
class btRigidBody;

namespace GopherEngine
{
    class ColliderComponent;
    class RigidBodyComponent;
    class Node;

    // A simple engine-facing identifier so game code does not need to know
    // anything about Bullet object pointers or internal storage.
    using ColliderId = std::uint32_t;
    constexpr ColliderId InvalidColliderId = 0;

    // Represents one overlap detected during the most recent physics update.
    struct CollisionPair
    {
        ColliderId a_;
        ColliderId b_;
    };

    // PhysicsWorld is the engine-facing wrapper around Bullet's collision-only API.
    // For this first pass it answers only one question: which colliders overlap?
    class PhysicsWorld : public Service<PhysicsWorld>
    {
        public:
            // Builds the Bullet collision world and the supporting objects it needs.
            PhysicsWorld();

            // Declared out-of-line so the header can forward-declare Bullet types.
            ~PhysicsWorld();

            // Synchronizes engine transforms into Bullet, runs collision detection,
            // and caches the overlap results for later queries this frame.
            void update(float delta_time);

            // Returns the list of overlap pairs recorded during the last update().
            const std::vector<CollisionPair>& get_collision_pairs() const;

            // Returns the owning node for a collider id, or nullptr if the id is unknown.
            Node* get_node_for_collider(ColliderId collider_id) const;

            // Returns the collider component for a collider id, or nullptr if the id is unknown.
            ColliderComponent* get_component_for_collider(ColliderId collider_id) const;

            // Creates and registers a Bullet sphere shape for a collider component.
            ColliderId register_sphere_collider(ColliderComponent* component, Node& node, float radius);

            // Creates and registers a Bullet box shape for a collider component.
            ColliderId register_box_collider(ColliderComponent* component, Node& node, const glm::vec3& size);

            // Removes a collider from the Bullet world and the engine registry.
            void unregister_collider(ColliderId collider_id);

            // Registers a dynamic rigid body on a node, creating a pending entry
            // if its collider has not been registered yet.
            ColliderId register_rigid_body(RigidBodyComponent* component, Node& node);

            // Removes rigid-body simulation from an already-registered collider.
            void unregister_rigid_body(ColliderId collider_id);


        private:
            // Stores the Bullet objects and engine back-pointers associated with
            // one engine collider.
            struct ColliderEntry
            {
                // Owns the Bullet collision shape used by this collider.
                std::unique_ptr<btCollisionShape> shape_;

                // Owns the Bullet collision object inserted into the world.
                std::unique_ptr<btCollisionObject> object_;

                // Points at the engine node whose world matrix drives this collider.
                Node* owner_node_{nullptr};

                // Points back at the engine component so collision flags can be updated.
                ColliderComponent* collider_component_{nullptr};

                // Points back at the rigid body component if one is registered.
                RigidBodyComponent* rigid_body_component_{nullptr};
            };

            // Shared helper used by the public collider-registration methods.
            ColliderId register_collider(
                ColliderComponent* component,
                Node& node,
                std::unique_ptr<btCollisionShape> shape);

            // Looks up or allocates the shared physics entry for a node.
            ColliderId get_or_create_entry_id(Node& node);

            // Removes whichever Bullet object is currently stored in the entry
            // and clears the local ownership pointer. This helper is narrow and
            // mechanical, so it improves readability without hiding the higher-
            // level registration state transitions from students.
            void remove_collision_object(ColliderEntry& entry);

            // Builds a Bullet rigid body using the fully populated data already
            // stored in a collider entry. This helper only constructs the
            // object; callers still decide when to remove old objects, insert
            // the new one into the world, and store ownership in the entry.
            std::unique_ptr<btRigidBody> build_rigid_body(ColliderId collider_id, const ColliderEntry& entry) const;

            // Builds a plain Bullet collision object using the fully populated
            // collider state already stored in an entry. Like build_rigid_body(),
            // this helper only constructs the object and leaves lifecycle
            // decisions to the calling register/unregister method.
            std::unique_ptr<btCollisionObject> build_collision_object(ColliderId collider_id, const ColliderEntry& entry) const;


            // Bullet object that stores low-level collision settings and factories.
            std::unique_ptr<btCollisionConfiguration> collision_configuration_;

            // Bullet object that creates and owns the narrow-phase contact manifolds.
            std::unique_ptr<btCollisionDispatcher> dispatcher_;

            // Bullet broadphase acceleration structure used to prune likely pairs.
            std::unique_ptr<btBroadphaseInterface> broadphase_;

            // Bullet solver used to resolve rigid-body constraints and contacts.
            std::unique_ptr<btConstraintSolver> solver_;

            // The collision-only Bullet world used in this assignment.
            std::unique_ptr<btDiscreteDynamicsWorld> dynamics_world_;

            // Maps engine collider ids to the Bullet objects and engine pointers they own.
            std::unordered_map<ColliderId, ColliderEntry> colliders_;

            // Cached overlap results from the most recent update.
            std::vector<CollisionPair> collision_pairs_;

            // Monotonic id generator for newly registered colliders.
            ColliderId next_collider_id_{1};
    };
}
