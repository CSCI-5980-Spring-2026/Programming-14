/* Progamming 14: Rigid Body Simulation II
 * CSCI 5980, Spring 2026, University of Minnesota
 * Instructor: Evan Suma Rosenberg <suma@umn.edu>
 */ 


#include <GopherEngine/Core/MainLoop.hpp>
#include <GopherEngine/Core/EventHandler.hpp>
#include <GopherEngine/Core/LightComponent.hpp>
#include <GopherEngine/Core/MeshComponent.hpp>
#include <GopherEngine/Core/OrbitControls.hpp>
#include <GopherEngine/Core/Utils.hpp>
#include <GopherEngine/Renderer/BlinnPhongMaterial.hpp>
#include <GopherEngine/Renderer/CellularNoiseMaterial.hpp>
#include <GopherEngine/Resource/MeshFactory.hpp>
#include <GopherEngine/Physics/BoxColliderComponent.hpp>
#include <GopherEngine/Physics/SphereColliderComponent.hpp>
#include <GopherEngine/Physics/RigidBodyComponent.hpp>
using namespace GopherEngine;

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include <memory>
#include <iostream>
using namespace std;

// A simple subclass of MainLoop to test that the main loop is working
// and the window, scene, and node classes are functioning correctly
class MainLoopTest: public MainLoop, public EventHandler
{
	public:
		// Constructor and destructor
		MainLoopTest();
		~MainLoopTest();

	private:

		// Override the pure virtual functions from MainLoop
		void configure() override;
		void initialize() override;
		void kinematic_update(float delta_time) override;
		void update(float delta_time) override;
		void on_key_press(const KeyEvent& event) override;

		// Helper functions to create nodes with meshes, colliders, and materials
		std::shared_ptr<Node> create_sphere_node(float radius);
		std::shared_ptr<Node> create_cube_node(glm::vec3 dimensions);
};

MainLoopTest::MainLoopTest() {

}

MainLoopTest::~MainLoopTest() {

}

// Helper function to create a node with a cube mesh, collider, and material
std::shared_ptr<Node> MainLoopTest::create_cube_node(glm::vec3 dimensions) {

	auto node = make_shared<Node>();

	auto mesh_component = make_shared<MeshComponent>();
	mesh_component->set_mesh(MeshFactory::create_cube(dimensions.x, dimensions.y, dimensions.z));
	node->add_component(mesh_component);

	auto collider = make_shared<BoxColliderComponent>(dimensions);
	node->add_component(collider);
	
	const glm::vec3 diffuse_color(Random::value(), Random::value(), Random::value());
	auto material = make_shared<BlinnPhongMaterial>();
	material->set_ambient_color(diffuse_color * 0.3f);
	material->set_diffuse_color(diffuse_color);
	material->set_specular_color(glm::vec3(1.5f));
	mesh_component->set_material(material);

	return node;
}

// Helper function to create a node with a sphere mesh, collider, and material
std::shared_ptr<Node> MainLoopTest::create_sphere_node(float radius) {

	auto node = make_shared<Node>();

	auto mesh_component = make_shared<MeshComponent>();
	mesh_component->set_mesh(MeshFactory::create_sphere(radius));
	node->add_component(mesh_component);

	auto collider = make_shared<SphereColliderComponent>(radius);
	node->add_component(collider);

	const glm::vec3 diffuse_color(Random::value(), Random::value(), Random::value());
	auto material = make_shared<CellularNoiseMaterial>();
	material->set_color(diffuse_color *glm::vec3(1.5f));
	mesh_component->set_material(material);

	return node;
}

// This function is called once at the beginning of the main loop, before the window is created.
// This means the OpenGL context is not yet available. It should be used for initial configuration.
void MainLoopTest::configure() {

	window_.set_title("CSCI 5980 Programming 14");
	window_.set_vertical_sync(true);
	window_.set_framerate_limit(60);
	renderer_.set_background_color(glm::vec4(0.5f, 0.75f, .9f, 1.f));
}

// This function is called once at the beginning of the main loop, after the window is created
// and the OpenGL context is available. It should be used for initializing the scene.
void MainLoopTest::initialize() {
	
	// Create default camera and set its initial position
	auto camera_node = scene_->create_default_camera();
	camera_node->look_at(glm::vec3(0.f,10.f, 30.f), glm::vec3(0.f, 0.f, 0.f));

	// Create a point light
	auto light_component = make_shared<LightComponent>(LightType::Point);
	light_component->get_light()->ambient_intensity_ = glm::vec3(0.2f, 0.2f, 0.2f);
	light_component->get_light()->diffuse_intensity_ = glm::vec3(1.f, 1.f, 1.f);
	light_component->get_light()->specular_intensity_ = glm::vec3(1.f, 1.f, 1.f);

	// Add the point light to the scene
	auto light_node = scene_->create_node();
	light_node->add_component(light_component);
	light_node->transform().position_ = glm::vec3(-10.f, 10.f, 10.f);

	constexpr float ground_size = 40.f;
	constexpr float ground_height = 2.f;
	constexpr float railing_height = 2.f;
	constexpr float railing_thickness = 1.f;
	const float railing_y = (ground_height + railing_height) * 0.5f;
	const float edge_offset = ground_size * 0.5f - railing_thickness * 0.5f;

	// Add a box for the ground
	auto ground_node = create_cube_node(glm::vec3(ground_size, ground_height, ground_size));
	ground_node->transform().position_ = glm::vec3(0.f, -ground_height/2, 0.f);
	scene_->add_node(ground_node);

	auto front_rail = create_cube_node(glm::vec3(ground_size, railing_height, railing_thickness));
	front_rail->transform().position_ = glm::vec3(0.f, railing_y, edge_offset);
	ground_node->add_child(front_rail);

	auto back_rail = create_cube_node(glm::vec3(ground_size, railing_height, railing_thickness));
	back_rail->transform().position_ = glm::vec3(0.f, railing_y, -edge_offset);
	ground_node->add_child(back_rail);

	auto left_rail = create_cube_node(glm::vec3(railing_thickness, railing_height, ground_size));
	left_rail->transform().position_ = glm::vec3(-edge_offset, railing_y, 0.f);
	ground_node->add_child(left_rail);

	auto right_rail = create_cube_node(glm::vec3(railing_thickness, railing_height, ground_size));
	right_rail->transform().position_ = glm::vec3(edge_offset, railing_y, 0.f);
	ground_node->add_child(right_rail);
}

// This function is called once per frame, before the physics system is updated.
// It should be used for updating the movement of objects that need to be processed for collision detection.
void MainLoopTest::kinematic_update(float delta_time) {

}

// This function is called once per frame, after the physics update and before rendering.
// Here, we can read and handle collision events from the physics world.
void MainLoopTest::update(float delta_time) {
}

void MainLoopTest::on_key_press(const KeyEvent& event) {
	constexpr glm::vec3 spawn_position(0.f, 15.f, 0.f);

	if (event.key == Key::Num1) {
		auto sphere_node = create_sphere_node(0.5f + Random::value());
		sphere_node->transform().position_ = spawn_position;
		scene_->add_node(sphere_node);
		sphere_node->add_component(make_shared<RigidBodyComponent>(1.f));
	}
	else if (event.key == Key::Num2) {
		auto cube_node = create_cube_node(glm::vec3(
			1.f + Random::value() * 2.f,
			1.f + Random::value() * 2.f,
			1.f + Random::value() * 2.f
		));
		cube_node->transform().position_ = spawn_position;
		scene_->add_node(cube_node);
		cube_node->add_component(make_shared<RigidBodyComponent>(1.f));
	}
}


int main()
{
	// Create an instance of the MainLoop subclass and start the main game loop
	MainLoopTest app;
	return app.run();
}
