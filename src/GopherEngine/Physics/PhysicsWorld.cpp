#include <GopherEngine/Physics/PhysicsWorld.hpp>

#include <GopherEngine/Core/Node.hpp>
#include <GopherEngine/Physics/ColliderComponent.hpp>
#include <GopherEngine/Physics/RigidBodyComponent.hpp>

#include <glm/gtc/quaternion.hpp>

#include <btBulletDynamicsCommon.h>

#include <algorithm>
#include <memory>
#include <stdexcept>

namespace
{
    // Converts an engine world matrix into the Bullet transform format used by
    // btCollisionObject. This keeps the GLM-to-Bullet translation in one place.
    btTransform to_bullet_transform(const glm::mat4& world_matrix)
    {
        btTransform transform;
        transform.setIdentity();

        // The translation lives in the fourth column of the engine world matrix.
        const glm::vec3 translation(world_matrix[3]);

        // The first three columns contain the transformed basis vectors.
        glm::vec3 x_axis = glm::vec3(world_matrix[0]);
        glm::vec3 y_axis = glm::vec3(world_matrix[1]);
        glm::vec3 z_axis = glm::vec3(world_matrix[2]);

        // If any basis vector is degenerate, fall back to the corresponding
        // world axis so quaternion conversion remains well-defined.
        if (glm::length(x_axis) == 0.f)
            x_axis = glm::vec3(1.f, 0.f, 0.f);
        if (glm::length(y_axis) == 0.f)
            y_axis = glm::vec3(0.f, 1.f, 0.f);
        if (glm::length(z_axis) == 0.f)
            z_axis = glm::vec3(0.f, 0.f, 1.f);

        // Normalize the basis vectors before converting them into a quaternion.
        // This removes any scale encoded in the world matrix so Bullet receives
        // only translation and rotation in its btTransform.
        const glm::mat3 rotation_matrix(
            glm::normalize(x_axis),
            glm::normalize(y_axis),
            glm::normalize(z_axis));

        const glm::quat rotation = glm::normalize(glm::quat_cast(rotation_matrix));

        // Bullet stores translation and rotation separately inside btTransform.
        transform.setOrigin(btVector3(translation.x, translation.y, translation.z));
        transform.setRotation(btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w));
        return transform;
    }

    btVector3 to_bullet_vector(const glm::vec3& vector)
    {
        return btVector3(vector.x, vector.y, vector.z);
    }

    void apply_bullet_transform_to_node(const btTransform& transform, GopherEngine::Node& node)
    {
        auto& node_transform = node.transform();

        const btVector3 origin = transform.getOrigin();
        node_transform.position_ = glm::vec3(origin.x(), origin.y(), origin.z());

        const btQuaternion rotation = transform.getRotation();
        node_transform.rotation_ = glm::normalize(glm::quat(
            rotation.w(),
            rotation.x(),
            rotation.y(),
            rotation.z()));
    }
}

namespace GopherEngine
{
    PhysicsWorld::PhysicsWorld()
    {
        // Bullet's collision world is assembled from several collaborating pieces:
        // configuration, dispatcher, broadphase, and finally the world itself.
        collision_configuration_ = std::make_unique<btDefaultCollisionConfiguration>();
        dispatcher_ = std::make_unique<btCollisionDispatcher>(collision_configuration_.get());
        broadphase_ = std::make_unique<btDbvtBroadphase>();
        solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
        dynamics_world_ = std::make_unique<btDiscreteDynamicsWorld>(
            dispatcher_.get(),
            broadphase_.get(),
            solver_.get(),
            collision_configuration_.get());
    }

    PhysicsWorld::~PhysicsWorld() = default;

    void PhysicsWorld::update(float delta_time)
    {
        // Start each frame with a fresh list of overlap pairs.
        collision_pairs_.clear();

        for (auto& [collider_id, entry] : colliders_)
        {
            // Clear each collider's cached overlap list before detecting the
            // current frame's overlaps.
            if (entry.collider_component_ != nullptr)
                entry.collider_component_->overlapping_colliders_.clear();

            // Skip incomplete entries rather than crashing; this keeps the
            // wrapper tolerant of registration teardown edge cases.
            if (entry.owner_node_ == nullptr || entry.object_ == nullptr)
                continue;

            // Copy the latest engine world transform into Bullet before running
            // collision detection.
            entry.object_->setWorldTransform(to_bullet_transform(entry.owner_node_->world_matrix()));
        }

        // Advance the dynamics world for this frame.
        dynamics_world_->stepSimulation(delta_time, 1);

        // Copy simulated transforms back into engine nodes. MainLoop will
        // rebuild matrices afterward using the updated node transforms.
        for (auto& [collider_id, entry] : colliders_)
        {
            if (!entry.rigid_body_component_ || entry.owner_node_ == nullptr || entry.object_ == nullptr)
                continue;

            // Copy the simulated transform back into the engine node.
            if (auto* rigid_body = btRigidBody::upcast(entry.object_.get()))
                apply_bullet_transform_to_node(rigid_body->getWorldTransform(), *entry.owner_node_);
        }

        const int manifold_count = dispatcher_->getNumManifolds();

        for (int manifold_index = 0; manifold_index < manifold_count; ++manifold_index)
        {
            // Each manifold represents the contact data Bullet found for one pair.
            auto* manifold = dispatcher_->getManifoldByIndexInternal(manifold_index);
            bool overlapping = false;

            for (int contact_index = 0; contact_index < manifold->getNumContacts(); ++contact_index)
            {
                // For this teaching pass, a contact with non-positive distance
                // counts as an overlap.
                if (manifold->getContactPoint(contact_index).getDistance() <= 0.f)
                {
                    overlapping = true;
                    break;
                }
            }

            // Ignore manifolds that do not currently represent an overlap.
            if (!overlapping)
                continue;

            // Recover the Bullet objects that participated in this overlap.
            const auto* object_a = static_cast<const btCollisionObject*>(manifold->getBody0());
            const auto* object_b = static_cast<const btCollisionObject*>(manifold->getBody1());

            // Each Bullet collision object stores the engine collider id in its
            // user index, so no separate reverse-lookup table is needed.
            const int object_a_id = object_a->getUserIndex();
            const int object_b_id = object_b->getUserIndex();

            if (object_a_id <= 0 || object_b_id <= 0)
                continue;

            const ColliderId collider_a = static_cast<ColliderId>(object_a_id);
            const ColliderId collider_b = static_cast<ColliderId>(object_b_id);

            // Store the pair in a consistent sorted order to simplify later queries.
            collision_pairs_.push_back({
                std::min(collider_a, collider_b),
                std::max(collider_a, collider_b)
            });

            // Mark both colliders as colliding so component-side queries stay simple.
            if (auto collider_it = colliders_.find(collider_a); collider_it != colliders_.end() &&
                collider_it->second.collider_component_ != nullptr)
                collider_it->second.collider_component_->overlapping_colliders_.push_back(collider_b);

            if (auto collider_it = colliders_.find(collider_b); collider_it != colliders_.end() &&
                collider_it->second.collider_component_ != nullptr)
                collider_it->second.collider_component_->overlapping_colliders_.push_back(collider_a);
        }
    }

    const std::vector<CollisionPair>& PhysicsWorld::get_collision_pairs() const
    {
        return collision_pairs_;
    }

    Node* PhysicsWorld::get_node_for_collider(ColliderId collider_id) const
    {
        const auto collider_it = colliders_.find(collider_id);
        if (collider_it == colliders_.end())
            return nullptr;

        return collider_it->second.owner_node_;
    }

    ColliderComponent* PhysicsWorld::get_component_for_collider(ColliderId collider_id) const
    {
        const auto collider_it = colliders_.find(collider_id);
        if (collider_it == colliders_.end())
            return nullptr;

        return collider_it->second.collider_component_;
    }

    ColliderId PhysicsWorld::register_sphere_collider(ColliderComponent* component, Node& node, float radius)
    {
        // SphereColliderComponent exposes radius in engine terms, so sanitize it
        // and build the Bullet sphere shape here during registration.
        const float clamped_radius = std::max(radius, 0.001f);
        return register_collider(
            component,
            node,
            std::make_unique<btSphereShape>(clamped_radius));
    }

    ColliderId PhysicsWorld::register_box_collider(ColliderComponent* component, Node& node, const glm::vec3& size)
    {
        // BoxColliderComponent exposes full size, but Bullet boxes are defined
        // by half-extents, so convert here inside the physics wrapper during registration.
        const glm::vec3 clamped_size = glm::max(size, glm::vec3(0.001f));
        return register_collider(
            component,
            node,
            std::make_unique<btBoxShape>(btVector3(
                clamped_size.x * 0.5f,
                clamped_size.y * 0.5f,
                clamped_size.z * 0.5f)));
    }

    void PhysicsWorld::unregister_collider(ColliderId collider_id)
    {
        // Ignore unknown ids so repeated teardown stays harmless.
        const auto collider_it = colliders_.find(collider_id);
        if (collider_it == colliders_.end())
            return;

        // Clear the overlap list so stale state is not left behind on the component.
        if (collider_it->second.collider_component_ != nullptr)
            collider_it->second.collider_component_->overlapping_colliders_.clear();

        auto& entry = collider_it->second;

        // Remove the Bullet object before stripping collider state. A rigid body
        // without a collision shape is not a meaningful teaching example here,
        // so the Bullet-side object must disappear until a collider is added back.
        if (entry.object_ != nullptr)
            remove_collision_object(entry);

        // Clear only the collider-owned data. If a rigid body component still
        // exists, we keep the entry as a pending record so re-adding a collider
        // later can reconstruct the Bullet object without losing the shared id.
        entry.collider_component_ = nullptr;
        entry.shape_.reset();

        // If the rigid body is also gone, nothing owns this entry anymore and it
        // can be removed from both registries.
        if (entry.rigid_body_component_ == nullptr)
            colliders_.erase(collider_it);
    }

    ColliderId PhysicsWorld::register_collider(
        ColliderComponent* component,
        Node& node,
        std::unique_ptr<btCollisionShape> shape)
    {
        if (component == nullptr)
            throw std::invalid_argument("Collider registration requires a valid component");

        const ColliderId collider_id = get_or_create_entry_id(node);
        auto& entry = colliders_.at(collider_id);

         if (entry.collider_component_ != nullptr)
            throw std::invalid_argument("Only one ColliderComponent is supported per node");

        // Store the collider-owned state on the shared node entry first. If a
        // rigid body was registered earlier, this fills in the missing shape and
        // lets us finish the Bullet-side object construction now.
        entry.shape_ = std::move(shape);
        entry.owner_node_ = &node;
        entry.collider_component_ = component;

        // Remove any stale Bullet object before rebuilding the node's physics
        // representation. This is most commonly a collision-only object that is
        // about to be replaced by a rigid body because the rigid body component
        // was registered earlier.
        if (entry.object_ != nullptr)
            remove_collision_object(entry);

        // If the node also has a rigid body component, complete the entry as a
        // dynamic rigid body. Otherwise, register the simpler collision-only
        // object used for overlap testing.
        if (entry.rigid_body_component_)
        {
            auto rigid_body = build_rigid_body(collider_id, entry);
            dynamics_world_->addRigidBody(rigid_body.get());
            entry.object_ = std::move(rigid_body);
            return collider_id;
        }

        auto object = build_collision_object(collider_id, entry);
        dynamics_world_->addCollisionObject(object.get());
        entry.object_ = std::move(object);

        return collider_id;
    }

     ColliderId PhysicsWorld::register_rigid_body(RigidBodyComponent* component, Node& node)
     {
        if (component == nullptr)
            throw std::invalid_argument("Rigid body registration requires a valid component");
        if (component->get_mass() <= 0.f)
            throw std::invalid_argument("RigidBodyComponent mass must be greater than zero");

        // Get or create the collider entry id
        const ColliderId collider_id = get_or_create_entry_id(node);
        auto& entry = colliders_.at(collider_id);

        if(entry.rigid_body_component_ != nullptr)
            throw std::invalid_argument("Only one RigidBodyComponent is supported per node");

        // Store the rigid body authoring data on the shared node entry first.
        // This is what makes registration order-independent: if the collider has
        // not been added yet, we still remember that the node wants to become a
        // dynamic rigid body once a collision shape exists.
        entry.rigid_body_component_ = component;

        // If a collision-only object already exists for this node, remove it
        // before building the rigid body. Bullet objects cannot change their
        // concrete type in place, so we must tear down the old object and then
        // create the new rigid-body representation explicitly.
        if (entry.object_ != nullptr)
            remove_collision_object(entry);

        // The collider's shape already exists, so we can now construct the
        // Bullet rigid body that will own mass and velocity state.
        auto rigid_body = build_rigid_body(collider_id, entry);
        dynamics_world_->addRigidBody(rigid_body.get());
        entry.object_ = std::move(rigid_body);
        return collider_id;
     }

     void PhysicsWorld::unregister_rigid_body(ColliderId collider_id)
     {
        const auto collider_it = colliders_.find(collider_id);
        if (collider_it == colliders_.end())
            return;

        auto& entry = collider_it->second;
        if (entry.rigid_body_component_ == nullptr)
            return;

        // If a Bullet object exists, remove it before changing the entry's role.
        // This is important for both complete entries and partially torn-down
        // ones: Bullet must not keep simulating an object whose engine-side
        // ownership has been removed.
        if (entry.object_ != nullptr)
            remove_collision_object(entry);

        // When the collider is still present, recreate the simpler collision-only
        // representation. This keeps overlap queries working after the rigid body
        // component has been removed.
        if (entry.collider_component_ != nullptr && entry.shape_ != nullptr && entry.owner_node_ != nullptr)
        {
            auto object = build_collision_object(collider_id, entry);
            dynamics_world_->addCollisionObject(object.get());
            entry.object_ = std::move(object);
            return;
        }

        // If neither component remains, the shared node entry is no longer useful.
        // Erasing the registry entry ensures a future component addition will
        // start with a fresh id and a fresh registry record.
        if (entry.collider_component_ == nullptr)
            colliders_.erase(collider_it);
     }

     ColliderId PhysicsWorld::get_or_create_entry_id(Node& node)
    {
        // The registry entry already stores its owning node pointer, so we can
        // recover an existing entry by scanning the registry directly. That keeps
        // the data model smaller and avoids maintaining a second map that mirrors
        // information already present in colliders_.
        for (const auto& [collider_id, entry] : colliders_)
        {
            if (entry.owner_node_ == &node)
                return collider_id;
        }

        const ColliderId collider_id = next_collider_id_++;
        colliders_.emplace(collider_id, ColliderEntry{});
        colliders_.at(collider_id).owner_node_ = &node;
        return collider_id;
    }

    void PhysicsWorld::remove_collision_object(ColliderEntry& entry)
    {
        if (entry.object_ == nullptr)
            return;

        if (auto* rigid_body = btRigidBody::upcast(entry.object_.get()))
            dynamics_world_->removeRigidBody(rigid_body);
        else
            dynamics_world_->removeCollisionObject(entry.object_.get());

        entry.object_.reset();
    }

    std::unique_ptr<btRigidBody> PhysicsWorld::build_rigid_body(ColliderId collider_id, const ColliderEntry& entry) const
    {
        if (entry.shape_ == nullptr || entry.owner_node_ == nullptr)
            throw std::runtime_error("Cannot build a rigid body without a collision shape and owner node");

         // This helper does one narrow job: construct a rigid body from the entry
        // data that has already been validated by the calling method. It does not
        // remove old Bullet objects or insert the new one into the world.
        btVector3 local_inertia(0.f, 0.f, 0.f);
        entry.shape_->calculateLocalInertia(entry.rigid_body_component_->get_mass(), local_inertia);

        btRigidBody::btRigidBodyConstructionInfo construction_info(
            entry.rigid_body_component_->get_mass(),
            nullptr,
            entry.shape_.get(),
            local_inertia);

        auto rigid_body = std::make_unique<btRigidBody>(construction_info);
        rigid_body->setWorldTransform(to_bullet_transform(entry.owner_node_->world_matrix()));
        rigid_body->setUserIndex(static_cast<int>(collider_id));
        return rigid_body;
    }

    std::unique_ptr<btCollisionObject> PhysicsWorld::build_collision_object(ColliderId collider_id, const ColliderEntry& entry) const
    {
        if (entry.shape_ == nullptr || entry.owner_node_ == nullptr)
            throw std::runtime_error("Cannot build a collision object without a collision shape and owner node");

        // This helper is the collision-only counterpart to build_rigid_body().
        // It constructs the Bullet object from the current entry state, while the
        // caller remains responsible for lifecycle and world-registration steps.
        auto object = std::make_unique<btCollisionObject>();
        object->setCollisionShape(entry.shape_.get());
        object->setWorldTransform(to_bullet_transform(entry.owner_node_->world_matrix()));
        object->setUserIndex(static_cast<int>(collider_id));
        return object;
    }
}
