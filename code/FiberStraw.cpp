#include "FiberStraw.h"

float dotProduct(glm::vec3 vec1, glm::vec3 vec2) {
	return (vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z);
}

glm::vec3 springforce(const glm::vec3 & P1, const glm::vec3 & V1, const glm::vec3 & P2, const glm::vec3 & V2, float L0, float ke, float kd)
{
	 
	 //-glm::vec3(ke*(getModule(P1-P2)-L0)+ dotProduct((kd *(V1-V2)), )
	//std::cout << getModule(P1 - P2) << std::endl;
	return -glm::vec3(((ke *(getModule(P1 - P2) - L0) + dotProduct((kd*(V1-V2)), ((P1-P2)/ getModule(P1 - P2)))) * ((P1 - P2) / getModule(P1 - P2))));
}
	
glm::vec3 computeForces(FiberStraw & fiber, int idx, const std::vector<ForceActuator*>& force_acts)
{
	glm::vec3 totalF = glm::vec3(0,0,0);

	std::vector<int> conn;
	for (int i = 0; i < fiber.connexions.size(); i++)
		if (fiber.connexions[i].first == idx) conn.push_back(fiber.connexions[i].second);

	for (int i = 0; i < conn.size(); i++)
	{
		if (abs(idx - conn[i])!=1)
			totalF += springforce(fiber.positions[idx], fiber.velocities[idx], fiber.positions[conn[i]], fiber.velocities[conn[i]], restLength * 2, ke/2.5f, kd);
		else
			totalF += springforce(fiber.positions[idx], fiber.velocities[idx], fiber.positions[conn[i]], fiber.velocities[conn[i]], restLength, ke, kd);
	}

	for (int i = 0; i < force_acts.size(); i++)
	{
		totalF += force_acts[i]->computeForce(mass, glm::vec3(0, 0, 0));
	}

	return totalF;
}

float getModule(glm::vec3& vec) {
	return abs(sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2)));
}

void normalize(glm::vec3& vec) {
	vec /= getModule(vec);
}

glm::vec3 getNormal(const glm::vec3 & vector1, const glm::vec3 & vector2)
{
	glm::vec3 normal;

	normal.x = vector1.y * vector2.z + vector2.y * vector1.z;
	normal.y = -(vector1.x * vector2.z + vector2.x * vector1.z);
	normal.z = vector1.x * vector2.y + vector2.x * vector1.y;

	return normal;
}

float getD(glm::vec3 normal, glm::vec3 point)
{
	return -(normal.x * point.x + normal.y * point.y + normal.z * point.z);
}

void verlet(float dt, FiberStraw & fiber, const std::vector<Collider*>& colliders, const std::vector<ForceActuator*>& force_acts)
{
	glm::vec3 aux;
	glm::vec3 aux2;
	for (int i = 1; i < NUM_PARTICLES; i++)
	{
			aux = fiber.positions[i];
			fiber.positions[i] = fiber.positions[i] + (fiber.positions[i] - fiber.lastPositions[i]) + (computeForces(fiber, i, force_acts) / mass) * pow(dt, 2);
			fiber.lastPositions[i] = aux;
			for (int j = 0; j < colliders.size(); j++)
				if (colliders[j]->checkCollision(fiber.lastPositions[i], fiber.positions[i]))
					colliders[j]->computeCollision(fiber.lastPositions[i], fiber.positions[i]);
		

			fiber.velocities[i] = (fiber.positions[i] - fiber.lastPositions[i]) / dt;
	}
}
