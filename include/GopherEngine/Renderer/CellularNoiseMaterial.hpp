#pragma once

#include <GopherEngine/Renderer/Material.hpp>

#include <memory>

namespace GopherEngine {

class CellularNoiseMaterial : public Material {
    public:
        CellularNoiseMaterial();
        void draw(const std::shared_ptr<Geometry>& geometry, const glm::mat4 &world_matrix, const RenderContext& context) override;

        void set_time(float time);
        float get_time() const;

        void set_color(const glm::vec3 &color);
        glm::vec3 get_color() const;
        
    private:
        float time_{1.f};
        glm::vec3 color_{1.f, 1.f, 1.f};
    };
}