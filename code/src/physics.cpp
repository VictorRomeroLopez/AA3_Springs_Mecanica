#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include "..\FiberStraw.h"

#define NUM_FIBRES 100

namespace Box {
	void drawCube();
}
namespace Axis {
	void drawAxis();
}
namespace Sphere {
	extern void updateSphere(glm::vec3 pos, float radius = 1.f);
	extern void drawSphere();
}
namespace Capsule {
	extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius = 1.f);
	extern void drawCapsule();
}
namespace Particles {
	extern const int maxParticles;
	extern void updateParticles(int startIdx, int count, float* array_data);
	extern void drawParticles(int startIdx, int count);
}
namespace Mesh {
	extern const int numCols;
	extern const int numRows;
	extern void updateMesh(float* array_data);
	extern void drawMesh();
}
namespace Fiber {
extern const int numVerts;
	extern void updateFiber(float* array_data);
	extern void drawFiber();
}
namespace Cube {
	extern void updateCube(const glm::mat4& transform);
	extern void drawCube();
}

// Boolean variables allow to show/hide the primitives
bool renderSphere = true;
bool renderCapsule = false;
bool renderParticles = false;
bool renderMesh = false;
bool renderFiber = true;
bool renderCube = false;

FiberStraw fiberS[100];
glm::vec3 SphereCenter(1, 3, 1);
float CurrentSphereRadius(1.0f);

//You may have to change this code
void renderPrims() {
	Box::drawCube();
	Axis::drawAxis();


	if (renderSphere)
	{
		Sphere::drawSphere();
		Sphere::updateSphere(SphereCenter, CurrentSphereRadius);
	}
	if (renderCapsule)
		Capsule::drawCapsule();

	if (renderParticles) {
		int startDrawingFromParticle = 0;
		int numParticlesToDraw = Particles::maxParticles;
		Particles::drawParticles(startDrawingFromParticle, numParticlesToDraw);
	}

	if (renderMesh)
		Mesh::drawMesh();
	if (renderFiber)
	{
		for (int i = 0; i < NUM_FIBRES; i++)
		{
			Fiber::drawFiber();
			Fiber::updateFiber(&fiberS[i].positions[0].x);
		}
	}

	if (renderCube)
		Cube::drawCube();
}

//Variables de la interfaz de usuario
bool playSimulation = false;
bool useSphereCollider = false;
float sphereY;
float sphereTurnRadius = 4.f;
float sphereTurnSpeed= 100.f;
float sphereRadius;
glm::vec2 k_Stretch;
glm::vec2 k_Bend;
float nu = 0.0f;
bool use_Gravity = true;
glm::vec3 gravity(0,-0.1f,0);
bool use_Wind = false;
glm::vec3 windForce(0.1,0,0);
float restLength = 1;
float mass = 0.1f;
float ke = 30;
float kd = 10.f;
bool reset=false;
float bounceCoefficient = 0.1f;
float particleLinkDistance;


std::vector<ForceActuator*> forces;
std::vector<Collider*> colliders;


void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	// Do your GUI code here....
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		ImGui::Checkbox("Play Simulation", &playSimulation);
		if (ImGui::Button("Reset"))
		{
			reset = true;
		}

		if (ImGui::TreeNode("Spring Parameters"))
		{
			ImGui::DragFloat2("K Stretch", &k_Stretch.x, 0.1f);
			ImGui::DragFloat2("K Bend", &k_Bend.x, 0.1f);
			ImGui::DragFloat("Particle Link Distance", &particleLinkDistance, 0.1f);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Elasticity and Friction"))
		{
			ImGui::DragFloat("Elastic Coeficient", &bounceCoefficient, 0.1f, 0, 1.0f);
			ImGui::DragFloat("Friction Coeficient", &nu, 0.1f, 0, 1.0f);
			ImGui::TreePop();
		}


		if(ImGui::TreeNode("Sphere Parameters"))
		{
			ImGui::Checkbox("Use Sphere Collider", &useSphereCollider);
			ImGui::DragFloat("Sphere Y", &SphereCenter.y, 0.1f);
			ImGui::DragFloat("Sphere Turn Radius", &sphereTurnRadius, 0.1f, 0.1f, 10.f);
			ImGui::DragFloat("Sphere Turn Speed", &sphereTurnSpeed, 0.1f);
			ImGui::DragFloat("Sphere Radius", &CurrentSphereRadius, 0.1f, 0.5f, 5.f);
			ImGui::TreePop();

		}
		

		if (ImGui::TreeNode("Forces"))
		{
			ImGui::Checkbox("Use Gravity", &use_Gravity);
			ImGui::DragFloat3("Gravity Accel", &gravity.x, 0.1f);
			ImGui::Checkbox("Use Wind", &use_Wind);
			ImGui::DragFloat3("Wind Accel", &windForce.x, 0.1f);
			ImGui::TreePop();
		}
		ImGui::DragFloat("Rest Length", &restLength, 0.01f, 0.0f, 3.f);
		ImGui::DragFloat("Mass", &mass, 0.01f, 0.1f, 10.f);
		ImGui::DragFloat("Ke", &ke, 0.01f, 0.1f, 100.f);
		ImGui::DragFloat("Kd", &kd, 0.01f, 0.1f, 10.f);
	}
	// .........................
	
	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	bool show_test_window = false;
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void rotateSphere(float dt) {
	if (getModule(SphereCenter - glm::vec3(0, 0, 0)) > 0.1f) {
		SphereCenter = (normalize(glm::vec3(SphereCenter.x, 0, SphereCenter.z)) * sphereTurnRadius) + glm::vec3(0.f, SphereCenter.y, 0.f);
		glm::vec3 centerRadius = normalize(glm::vec3(SphereCenter.z, 0, -SphereCenter.x)) * sphereTurnSpeed * dt;
		SphereCenter = normalize(glm::vec3(SphereCenter.x + centerRadius.x, 0, SphereCenter.z + centerRadius.z) - glm::vec3(0, 0, 0)) * sphereTurnRadius + glm::vec3(0.f, SphereCenter.y, 0.f);
	}
}

void PhysicsInit() {
	colliders.push_back(new PlaneCol(std::vector<glm::vec3>{glm::vec3(5, 0, -5), glm::vec3(5, 10, -5), glm::vec3(-5, 10, -5)}));
	colliders.push_back(new PlaneCol(std::vector<glm::vec3>{glm::vec3(-5, 0, 5), glm::vec3(-5, 0, -5), glm::vec3(5, 0, -5)}));
	colliders.push_back(new PlaneCol(std::vector<glm::vec3>{glm::vec3(5, 0, -5), glm::vec3(5, 0, 5), glm::vec3(5, 10, 5)}));
	colliders.push_back(new PlaneCol(std::vector<glm::vec3>{glm::vec3(-5, 0, 5), glm::vec3(5, 0, 5), glm::vec3(5, 10, 5)}));
	colliders.push_back(new PlaneCol(std::vector<glm::vec3>{glm::vec3(-5, 0, -5), glm::vec3(-5, 10, -5), glm::vec3(-5, 0, 5)}));
	colliders.push_back(new PlaneCol(std::vector<glm::vec3>{glm::vec3(-5, 10, 5), glm::vec3(-5, 10, -5), glm::vec3(5, 10, -5)}));
	colliders.push_back(new SphereCol(SphereCenter, CurrentSphereRadius));
	forces.push_back(new WindForce());
	forces.push_back(new GravityForce());
}

void PhysicsUpdate(float dt) {
	
	colliders.pop_back();
	colliders.push_back(new SphereCol(SphereCenter, CurrentSphereRadius));

	dt /= 10;
	for (int i = 0; i < NUM_FIBRES; i++)
	{
		for(int j=0;j<10;j++)
		verlet(dt, fiberS[i], colliders, forces);
	}

	if (reset)
	{
		reset = false;
		for (int i = 0; i < NUM_FIBRES; i++)
			fiberS[i] = FiberStraw();
	}
	rotateSphere(dt);
}

void PhysicsCleanup() {
	// Do your cleanup code here...
	// ............................
}

