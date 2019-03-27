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

struct _Fiber
{
	glm::vec3 positions[5];
	glm::vec3 velocities[5];
	std::vector<std::pair<int, int>> connexions;
	_Fiber() {
		float x = (float)((rand() % 9000) - 4500) / 1000;
		float z = (float)((rand() % 9000) - 4500) / 1000;
		for (int i = 0; i < NUM_PARTICLES; i++) {
			positions[i] = glm::vec3(x, i-1, z);
		}
		for (int i = 0; i < NUM_PARTICLES; i++) {
			velocities[i] = glm::vec3((float)((rand() % 10) - 5), 0, (float)((rand() % 10) - 5));
		}

		for (int i = 0; i < NUM_PARTICLES; i++)
		{
			std::cout << i << std::endl;
			for (int j = -2; j <3; j++)
			{
				if (i + j >= 0 && i + j < NUM_PARTICLES && j != 0)
				{
					connexions.push_back(std::pair<int, int>{i, i + j});
					std::cout << i + j;
				}
			}
			std::cout<< std::endl;
		}
	}
};

struct FiberStraw
{
	
	_Fiber fibers[100];

	float particleMass;

	FiberStraw()
	{

	}
	
};