#version 460 core

layout(std140, binding = 0) uniform Matrices
{
    mat4 projection_matrix;
    mat4 model_view_matrix;
    mat4 model_matrix;
    mat4 view_matrix;
    mat4 normal_matrix;
    vec4 eye_position;
};

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec4 color;
layout(location = 3) in vec2 uv;

out vec2 tex_coord;

void main() 
{
    tex_coord = uv.xy;
    gl_Position = projection_matrix * model_view_matrix * vec4(position, 1);
}