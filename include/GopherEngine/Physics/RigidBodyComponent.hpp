#pragma once

#include <GopherEngine/Core/Component.hpp>
#include <GopherEngine/Physics/PhysicsWorld.hpp>

namespace GopherEngine
{
    class RigidBodyComponent : public Component
    {
        public:
             explicit RigidBodyComponent(float mass = 1.f);
            ~RigidBodyComponent() override;

            void initialize(Node& node) override;

            float get_mass() const;

        private:
            float mass_{1.f};
            ColliderId collider_id_{InvalidColliderId};
    };
}