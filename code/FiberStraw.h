#pragma once

#include <stdlib.h>
#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <time.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>

#define NUM_PARTICLES 5
extern bool playSimulation;
extern float restLength;
extern float mass;
extern glm::vec2 ke;
extern glm::vec2 kd;
extern float bounceCoefficient;
extern glm::vec3 gravity;
extern glm::vec3 windForce;
extern bool use_Gravity;
extern bool use_Wind;

glm::vec3 getNormal(const glm::vec3 & vector1, const glm::vec3 & vector2);

float getD(glm::vec3 normal, glm::vec3 point);

float getModule(glm::vec3& vec);

void normalize(glm::vec3& vec);

float dotProduct(glm::vec3 vec1, glm::vec3 vec2);

struct FiberStraw
{
	glm::vec3 lastPositions[5];
	glm::vec3 positions[5];
	glm::vec3 velocities[5];
	std::vector<std::pair<int, int>> connexions;
	FiberStraw() {
		float x = (float)((rand() % 9000) - 4500) / 1000;
		float z = (float)((rand() % 9000) - 4500) / 1000;
		for (int i = 0; i < NUM_PARTICLES; i++) {
			positions[i] = glm::vec3(x, i - 0.9f, z);
			lastPositions[i] = positions[i];
		}
		for (int i = 0; i < NUM_PARTICLES; i++) {
			velocities[i] = glm::vec3(0, 0, 0);
		}

		for (int i = 0; i < NUM_PARTICLES; i++)
		{
			for (int j = -2; j < 3; j++)
			{
				if (i + j >= 0 && i + j < NUM_PARTICLES && j != 0)
				{
					connexions.push_back(std::pair<int, int>{i, i + j});

				}
			}			
		}
	}
	
};

struct ForceActuator { virtual glm::vec3 computeForce(float mass, const glm::vec3& position) = 0;};

struct GravityForce : ForceActuator
{
	glm::vec3 computeForce(float mass, const glm::vec3& position)
	{
		if(use_Gravity)
			return gravity * mass;
		return glm::vec3(0, 0, 0);
	}
};

struct WindForce : ForceActuator
{
	glm::vec3 computeForce(float mass, const glm::vec3& position)
	{
		if(use_Wind)
			return windForce * mass;
		return glm::vec3(0, 0, 0);
	}
};

glm::vec3 springforce(const glm::vec3& P1, const glm::vec3& V1, const glm::vec3& P2, const glm::vec3& V2, float L0, float ke, float kd);

glm::vec3 computeForces(FiberStraw& fiber, int idx, const std::vector<ForceActuator*>& force_acts);

struct Collider {
	virtual bool checkCollision(const glm::vec3& prev_pos,	const glm::vec3& next_pos) = 0;
	virtual void getPlane(glm::vec3& normal, float& d) = 0;
	void computeCollision(glm::vec3& old_pos, glm::vec3& new_pos) {
		glm::vec3 normal;
		float d;
		getPlane(normal, d);

		glm::vec3 temp_new_pos = glm::vec3(new_pos - 1 * (glm::dot(normal, new_pos) + d) * normal);
		glm::vec3 temp_old_pos = glm::vec3(old_pos - 1 * (glm::dot(normal, old_pos) + d) * normal);
		glm::vec3 temp = temp_new_pos - temp_old_pos;
		new_pos = new_pos - (1 + bounceCoefficient) * (glm::dot(normal, new_pos) + d) * normal;
		old_pos = (old_pos - (1 + bounceCoefficient) * (glm::dot(normal, old_pos) + d) * normal) + (temp*0.2f);
	}
};
struct PlaneCol : Collider {
	glm::vec3 point, vector1, vector2;
	
	PlaneCol(std::vector<glm::vec3> points) {
		point = points[0];
		vector1 = points[1] - points[0];
		vector2 = points[2] - points[0];
	}
	void getPlane(glm::vec3& _normal, float& d)
	{
		_normal = getNormal(vector1, vector2);
		normalize(_normal);
		d = getD(_normal, point);
	}

	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos)
	{
		glm::vec3 normal;
		float d;
		getPlane(normal, d);
		float value_prev_pos = normal.x * prev_pos.x + normal.y * prev_pos.y + normal.z * prev_pos.z + d;
		float value_next_pos = normal.x * next_pos.x + normal.y * next_pos.y + normal.z * next_pos.z + d;
		return(value_prev_pos <= 0 && value_next_pos > 0) || (value_prev_pos >= 0 && value_next_pos < 0);

	}

};
struct SphereCol : Collider {
	
	glm::vec3 old_pos;
	glm::vec3 new_pos;
	glm::vec3 center;
	float radius;

	SphereCol(glm::vec3 _center, float _radius) : center{ _center }, radius{ _radius }, old_pos{ glm::vec3{0,0,0} }, new_pos{ glm::vec3{0,0,0} }{ };

	bool checkCollision(const glm::vec3& prev_pos, const glm::vec3& next_pos) {
		old_pos = prev_pos;
		new_pos = next_pos;
		return getModule(next_pos - center) < radius;
	}

	void getPlane(glm::vec3& normal, float& d) {
		glm::vec3 v = new_pos - old_pos;
		float a = dotProduct(v, v);
		float b = 2 * dotProduct(old_pos, v) + 2 * dotProduct(v, center);
		float c = dotProduct(old_pos, old_pos) - 2 * dotProduct(old_pos, center) + dotProduct(center, center);
		float landa1 = (-b + sqrt(pow(b, 2) - 4 * a * c)) / 2 * a;
		float landa2 = (-b - sqrt(pow(b, 2) - 4 * a * c)) / 2 * a;
		glm::vec3 punt_tall;

		if (landa1 < landa2)
			punt_tall = old_pos + landa1 * v;
		else
			punt_tall = old_pos + landa2 * v;

		normal = punt_tall - center;
		normalize(normal);
		d = getD(normal, punt_tall);
	}
};

void verlet(float dt, FiberStraw& fiber, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts);