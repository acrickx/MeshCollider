#include "vcl/vcl.hpp"
#include <iostream>
#include <chrono>

#include "simulation.hpp"

using namespace vcl;

struct gui_parameters {
	bool display_frame = true;
	bool add_sphere = true;
	bool add_obj = true;
	bool reverse_gravity = false;
};

struct user_interaction_parameters {
	vec2 mouse_prev;
	timer_fps fps_record;
	mesh_drawable global_frame;
	gui_parameters gui;
	bool cursor_on_gui;

};
user_interaction_parameters user;

struct scene_environment
{
	camera_around_center camera;
	mat4 projection;
	vec3 light;
};
scene_environment scene;

segments_drawable cube_wireframe;

model object;
mesh_drawable object_drawable;
model pin;
mesh_drawable pin_drawable;
mesh_drawable sphere;
mesh_drawable xwing_drawable;
rotation rot(vec3(1, 0, 0), pi / 2.f);


timer_event_periodic timer(3.f);
//timer_event_periodic timer(0.01f);
std::vector<particle_structure> particles;
std::vector<model*> objects;

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void window_size_callback(GLFWwindow* window, int width, int height);

void initialize_data(int scenario);
void display_scene();
void display_scene_obj();
void display_scene_both();
void display_interface();
void display_interface_obj();
void display_interface_both();
void emit_particle(int scenario);
void emit_object(const model& object, int scenario);

int main(int, char* argv[])
{
	std::cout << "Run " << argv[0] << std::endl;

	int scenario = 3;

	bool simu_xwing = false;
	if (scenario == 3)
		user.gui.add_obj = false;
	size_t nbPart = 0; // for execution time
	int const width = 1280, height = 1024;
	GLFWwindow* window = create_window(width, height);
	window_size_callback(window, width, height);
	std::cout << opengl_info_display() << std::endl;;

	imgui_init(window);
	glfwSetCursorPosCallback(window, mouse_move_callback);
	glfwSetWindowSizeCallback(window, window_size_callback);

	std::cout << "Initialize data ..." << std::endl;
	initialize_data(scenario);

	std::cout << "Start animation loop ..." << std::endl;
	user.fps_record.start();
	timer.start();
	glEnable(GL_DEPTH_TEST);
	while (!glfwWindowShouldClose(window))
	{
		scene.light = scene.camera.position();
		user.fps_record.update();
		timer.update();

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_DEPTH_BUFFER_BIT);
		imgui_create_frame();
		if (user.fps_record.event) {
			std::string const title = "VCL Display - " + str(user.fps_record.fps) + " fps";
			glfwSetWindowTitle(window, title.c_str());
		}

		ImGui::Begin("GUI", NULL, ImGuiWindowFlags_AlwaysAutoResize);
		user.cursor_on_gui = ImGui::IsAnyWindowFocused();
		if (user.gui.display_frame) draw(user.global_frame, scene);

		// 0 : balls emitted in the canyon
		// 1 : rain in the canyon
		if (scenario == 0 || scenario == 1) {
			emit_particle(scenario);
			display_interface();
			float const dt = 0.005f * timer.scale;
			simulate(particles, dt, objects);
			display_scene();
		}

		// 2 : pins in the canyon
		if (scenario == 2) {
			emit_object(pin, scenario);
			display_interface_obj();
			float const dt = 0.005f * timer.scale;
			simulate(objects, dt, false);
			display_scene_obj();
		}

		// 3 : object rising in a pool of balls
		if (scenario == 3) {
			//auto start = std::chrono::high_resolution_clock::now();
			emit_particle(scenario);
			float const dt = 0.005f * timer.scale;
			if (user.gui.reverse_gravity || simu_xwing) {
				simu_xwing = true;
				simulate(objects, dt, user.gui.reverse_gravity);
			}
			simulate(particles, dt, objects);
			/*if (particles.size() != nbPart && particles.size() % 30 == 2 && particles.size() < 315) {
				nbPart = particles.size();
				auto stop = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
				std::cout << "nb spheres : " << particles.size() << " ";
				std::cout << "Execution time : " << duration.count() << " microseconds." << std::endl << std::endl;
			}*/
			display_scene_both();
			display_interface_both();
		}

		ImGui::End();
		imgui_render_frame(window);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	imgui_cleanup();
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void emit_object(const model& mod, int scenario) {
	static buffer<vec3> const color_lut = { {1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1} };
	if (timer.event && user.gui.add_obj) {
		float const theta = rand_interval(0, 2 * pi);
		vec3 const v = vec3(1.0f * std::cos(theta), 1.0f * std::sin(theta), 5.f);
		model* obj = new model(mod);
		obj->position() = { 0, 0, 0 };
		obj->color() = color_lut[int(rand_interval() * color_lut.size())];
		obj->velocity() = v;
		obj->mass() = 1.0f;
		objects.push_back(obj);
	}
}

void emit_particle(int scenario)
{
	// Emit particle with random velocity
	// Assume first that all particles have the same radius and mass
	static buffer<vec3> const color_lut = { {1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1} };
	if (timer.event && user.gui.add_sphere) {
		vec3 pos;
		vec3 v;
		float rad = 0;
		float mass = 0;
		if (scenario == 0) {
			float const theta = rand_interval(0, 2 * pi);
			v = vec3(1.0f * std::cos(theta), 1.0f * std::sin(theta), 4.0f);
			rad = 0.02f;
			mass = 1.f;
		}
		else if (scenario == 1) {
			pos = vec3(rand_interval(-0.97f, 0.97f), rand_interval(-0.97f, 0.97f), rand_interval(0.9f, 0.97f));
			if (particles.size() == 0)
				rad = 0.02f;
			else
				rad = rand_interval(0.01f, 0.02f);
			mass = rad * 50;
		}
		else if (scenario == 3) {
			float const theta = rand_interval(0, 2 * pi);
			v = vec3(1.0f * std::cos(theta), 1.0f * std::sin(theta), 4.0f);
			rad = 0.08f;
			mass = 1.f;
		}
		particle_structure particle;
		particle.p = pos;
		particle.r = rad;
		particle.c = color_lut[int(rand_interval() * color_lut.size())];
		particle.v = v;
		particle.m = mass;
		particles.push_back(particle);
	}
}

void initialize_data(int scenario)
{
	GLuint const shader_mesh = opengl_create_shader_program(opengl_shader_preset("mesh_vertex"), opengl_shader_preset("mesh_fragment"));
	GLuint const shader_uniform_color = opengl_create_shader_program(opengl_shader_preset("single_color_vertex"), opengl_shader_preset("single_color_fragment"));
	GLuint const texture_white = opengl_texture_to_gpu(image_raw{ 1,1,image_color_type::rgba,{255,255,255,255} });
	mesh_drawable::default_shader = shader_mesh;
	mesh_drawable::default_texture = texture_white;
	curve_drawable::default_shader = shader_uniform_color;
	segments_drawable::default_shader = shader_uniform_color;

	user.global_frame = mesh_drawable(mesh_primitive_frame());
	user.gui.display_frame = false;
	scene.camera.distance_to_center = 2.5f;
	scene.camera.look_at({ 4,3,2 }, { 0,0,0 }, { 0,0,1 });

	//sphere model
	sphere = mesh_drawable(mesh_primitive_sphere());

	//canyon mesh
	//object = model(mesh_load_file_obj("../MeshCollider/assets/canyon2.obj"));		
	//object.scale(0.002f);
	//object.translate(vec3(0.f, 0.f, -1.f));
	//object.BVHroot() = BVHnode(&(object.modelMesh()), 1.f);
	//objects.push_back(&object);
	//object_drawable = mesh_drawable(object.modelMesh());
	//GLuint texture_id = opengl_texture_to_gpu(image_load_png("../MeshCollider/assets/diffuse.png"));
	//object_drawable.texture = texture_id;

	//tyra mesh
	//object = model(mesh_load_file_obj("../MeshCollider/assets/tyra.obj"));
	//object.rotate(rot);	
	////object.translate(vec3(0.f, 0.f, -1.f));
	//object.BVHroot() = BVHnode(&(object.modelMesh()), 1.f);
	//objects.push_back(&object);

	//obstacle mesh
	if (scenario == 0 || scenario == 1 || scenario == 2) {
		object = model(mesh_load_file_obj("../MeshCollider/assets/canyon2.obj"));
		object.scale(0.002f);
		object.translate(vec3(0.f, 0.f, -1.f));
		object.BVHroot() = BVHnode(&(object.modelMesh()), 1.f);
		objects.push_back(&object);
		object_drawable = mesh_drawable(object.modelMesh());
		GLuint texture_id = opengl_texture_to_gpu(image_load_png("../MeshCollider/assets/diffuse.png"));
		object_drawable.texture = texture_id;
	}

	if (scenario == 2 || scenario == 3) {
		pin = model(mesh_load_file_obj("../MeshCollider/assets/bowling_pin.obj"));
		pin.rotate(rot);
		pin.scale(0.04f);
		pin.BVHroot() = BVHnode(&(pin.modelMesh()), 1.f);
		pin_drawable = mesh_drawable(pin.modelMesh());
	}	

	if (scenario == 3) {
		model* obj = new model(pin);
		obj->updatePosition(vec3(1000.f, -1000.f, -1000.f));
		objects.push_back(obj);
		//xwing mesh		
		model* xwing = new model(mesh_load_file_obj("../MeshCollider/assets/xwing.obj"));
		xwing->rotate(rot);
		xwing->scale(0.13f);
		xwing->translate(vec3(0.f, 0.2f, -0.75f));
		xwing->BVHroot() = BVHnode(&(xwing->modelMesh()), 1.f);
		objects.push_back(xwing);
		xwing_drawable = mesh_drawable(xwing->modelMesh());
	}

	// Edges of the containing cube
	//  Note: this data structure is set for display purpose - don't use it to compute some information on the cube - it would be un-necessarily complex
	buffer<vec3> cube_wireframe_data = { {-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
		{-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
		{-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1} };
	cube_wireframe = segments_drawable(cube_wireframe_data);
}

void display_scene()
{
	size_t const N = particles.size();
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure const& particle = particles[k];
		sphere.shading.color = particle.c;
		sphere.transform.translate = particle.p;
		sphere.transform.scale = particle.r;
		draw(sphere, scene);
	}
	draw(object_drawable, scene);
	draw(cube_wireframe, scene);
}

//start k = 1 to avoid the ground
void display_scene_obj()
{
	size_t const N = objects.size();
	for (size_t k = 1; k < N; k++)
	{
		const model* obj = objects[k];
		pin_drawable.shading.color = obj->color();
		pin_drawable.transform.translate = obj->position();
		pin_drawable.transform.scale = obj->sizeScale();
		draw(pin_drawable, scene);
	}
	draw(object_drawable, scene);
	draw(cube_wireframe, scene);
}

void display_scene_both()
{
	size_t const N = particles.size();
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure const& particle = particles[k];
		sphere.shading.color = particle.c;
		sphere.transform.translate = particle.p;
		sphere.transform.scale = particle.r;
		draw(sphere, scene);
	}
	size_t const N_obj = objects.size();
	for (size_t k = 0; k < N_obj; k++)
	{
		const model* obj = objects[k];
		//xwing_drawable.shading.color = obj->color();
		xwing_drawable.transform.translate = obj->position();
		xwing_drawable.transform.scale = obj->sizeScale();
		draw(xwing_drawable, scene);
	}
	draw(cube_wireframe, scene);
}

void display_interface()
{
	ImGui::Checkbox("Frame", &user.gui.display_frame);
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
	ImGui::SliderFloat("Interval create object", &timer.event_period, 0.05f, 2.0f, "%.2f s");
	ImGui::Checkbox("Add sphere", &user.gui.add_sphere);
}

void display_interface_obj()
{
	ImGui::Checkbox("Frame", &user.gui.display_frame);
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
	ImGui::SliderFloat("Interval create object", &timer.event_period, 0.05f, 2.0f, "%.2f s");
	ImGui::Checkbox("Add object", &user.gui.add_obj);
}

void display_interface_both()
{
	ImGui::Checkbox("Frame", &user.gui.display_frame);
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
	ImGui::SliderFloat("Interval create object", &timer.event_period, 0.05f, 2.0f, "%.2f s");
	ImGui::Checkbox("Add sphere", &user.gui.add_sphere);
	ImGui::Checkbox("Reverse gravity", &user.gui.reverse_gravity);
}

void window_size_callback(GLFWwindow*, int width, int height)
{
	glViewport(0, 0, width, height);
	float const aspect = width / static_cast<float>(height);
	scene.projection = projection_perspective(50.0f * pi / 180.0f, aspect, 0.1f, 100.0f);
}


void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
	vec2 const  p1 = glfw_get_mouse_cursor(window, xpos, ypos);
	vec2 const& p0 = user.mouse_prev;
	glfw_state state = glfw_current_state(window);

	auto& camera = scene.camera;
	if (!user.cursor_on_gui) {
		if (state.mouse_click_left && !state.key_ctrl)
			scene.camera.manipulator_rotate_trackball(p0, p1);
		if (state.mouse_click_left && state.key_ctrl)
			camera.manipulator_translate_in_plane(p1 - p0);
		if (state.mouse_click_right)
			camera.manipulator_scale_distance_to_center((p1 - p0).y);
	}

	user.mouse_prev = p1;
}

void opengl_uniform(GLuint shader, scene_environment const& current_scene)
{
	opengl_uniform(shader, "projection", current_scene.projection);
	opengl_uniform(shader, "view", scene.camera.matrix_view());
	opengl_uniform(shader, "light", scene.light, false);
}