
#include <GopherEngine/Renderer/CellularNoiseMaterial.hpp>
#include <GopherEngine/Renderer/ResourceUploader.hpp>
#include <GopherEngine/Renderer/ShaderLoader.hpp>

#include <glm/gtc/type_ptr.hpp> // value_ptr function to convert glm types to pointer for OpenGL

#include <GL/glew.h>          // Must come first
#include <SFML/OpenGL.hpp>    // After glew

#include <memory>
using namespace std;

namespace GopherEngine {

    struct CellularNoiseUBO
    {
        glm::vec4 color;    // vec3 color + 1.0 alpha
        float time;         // Time value for animating the noise
        float padding[3];   // pad to 16-byte boundary
    };

    CellularNoiseMaterial::CellularNoiseMaterial() {
        
    }

    void CellularNoiseMaterial::draw(const shared_ptr<Geometry>& geometry, const glm::mat4 &world_matrix, const RenderContext& context) {

         // If the resource is not a mesh, we don't know how to draw it, so just return early.
        // This allows the renderer to support multiple resource types in the future 
        // without needing to change the Material interface (e.g., points, particle systems, etc.)
        if (geometry->type_ != GeometryType::Mesh)
            return;
        
        // Downcast to the resource to a mesh reference. This is safe because we checked the type above.
        const shared_ptr<Mesh> mesh = static_pointer_cast<Mesh>(geometry);

        // Skip drawing if mesh has no geometry
        if(mesh->array_buffer_.empty() || mesh->element_buffer_.empty())
            return;

        // Lazy upload to GPU on first draw
        if(mesh->vao_ == 0) {

            // If the mesh is not already pending upload, enqueue it for upload
            // and return to avoid drawing before GPU resources are ready
            if(!mesh->pending_upload_) 
                Service<ResourceUploader>::get().upload_mesh(mesh);
           
            return;
        }

        // Retrieve and bind the shader program
        auto shader_program = ShaderLoader::get().load_shader(ShaderType::CellularNoise, sizeof(CellularNoiseUBO));
        bind_shader(*shader_program, world_matrix, context, false, false);

        // Update material uniform buffer object (UBO)
        CellularNoiseUBO ubo;
        ubo.color = glm::vec4(color_, 1.f);
        ubo.time = time_;

        // Upload the material UBO data to the GPU
        glBindBuffer(GL_UNIFORM_BUFFER, shader_program->get_material_ubo());
        glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(CellularNoiseUBO), &ubo);

        // Bind the vertex array and draw the mesh
        glBindVertexArray(mesh->vao_);
        glDrawElements(GL_TRIANGLES, mesh->element_buffer_.size(), GL_UNSIGNED_INT, 0);

        // Unbind the shader and vertex array to clean up state
        unbind_shader(*shader_program);

    }

    void CellularNoiseMaterial::set_time(float time) {
        time_ = time;
    }

    float CellularNoiseMaterial::get_time() const {
        return time_;
    }

    void CellularNoiseMaterial::set_color(const glm::vec3 &color) {
        color_ = color;
    }

    glm::vec3 CellularNoiseMaterial::get_color() const {
        return color_;
    }

} 