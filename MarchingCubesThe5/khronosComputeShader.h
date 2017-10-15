#pragma once
#include <GL/glew.h>
#include <GL/glut.h>

struct pos
{
	float x, y, z, w;
	// positions
};

struct vel
{
	float vx, vy, vz, vw;
	// velocities
};

struct color
{
	float r, g, b, a;
	// colors
};

// need to do the following for both position, velocity, and colors of the particles :
class khronosComputeShader {
public:
	GLuint  posSSbo;
	GLuint  velSSbo;

	int NUM_PARTICLES = 1024 * 1024;
	int WORK_GROUP_SIZE = 128;

public:

	void initExample() {
		glGenBuffers(1, &posSSbo);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, posSSbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, NUM_PARTICLES * sizeof(struct pos), NULL, GL_STATIC_DRAW);
		GLint bufMask = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT;

		// the invalidate makes a big difference when re-writing
		struct pos *points = (struct pos *) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, NUM_PARTICLES * sizeof(struct pos), bufMask);
		for (int i = 0; i < NUM_PARTICLES; i++)
		{
			float distance = 5.0f;

			int xDirection = 10, zDirection = 10; //

			int i_x = i % xDirection;
			int i_z = i / xDirection % zDirection;
			int i_y = i / (xDirection * zDirection);

			points[i].x = (float)i_x * distance;
			points[i].y = (float)i_y * distance;
			points[i].z = (float)i_z * distance;
			points[i].w = 1.;
		}

		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
		glGenBuffers(1, &velSSbo);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, velSSbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, NUM_PARTICLES * sizeof(struct vel), NULL, GL_STATIC_DRAW);
		struct vel *vels = (struct vel *) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, NUM_PARTICLES * sizeof(struct vel), bufMask);

		for (int i = 0; i < NUM_PARTICLES; i++)
		{
			vels[i].vx = 0.0f;
			vels[i].vy = -0.1f;
			vels[i].vz = 0.0f;
			vels[i].vw = 0.;
		}

		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	void beforeCS() {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, posSSbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, velSSbo);
	
	}

	void afterCS() {
		glDispatchCompute(NUM_PARTICLES / WORK_GROUP_SIZE, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}

	void afterVsFS() {
		glBindBuffer(GL_ARRAY_BUFFER, posSSbo);
		glVertexPointer(4, GL_FLOAT, 0, (void *)0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glDrawArrays(GL_POINTS, 0, NUM_PARTICLES);
		glDisableClientState(GL_VERTEX_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
};