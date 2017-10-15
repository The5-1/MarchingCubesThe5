#ifndef SKYBOX_H
#define SKYBOX_H
/*
http://antongerdelan.net/opengl/cubemaps.html
Advantage: Uses stb_image.h
*/

/* *********************************************************************************************************
Important for skybox:
1. Disable depth masking when drawing, because skybox will always be closest to camera
2. Skybox must be the first thing that you draw in your scene
3. Enable depth mask for rest of scene
********************************************************************************************************* */

#include <GL/glew.h>
#include <GL/glut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
using namespace std;
/* *********************************************************************************************************
Skybox vertex does not need normal, tc, ...
********************************************************************************************************* */
struct VertexSkybox {
	glm::vec3 Position;
	VertexSkybox() {
	}
	VertexSkybox(glm::vec3 _Position) {
		this->Position = _Position;
	}
};

/* *********************************************************************************************************
Skybox
********************************************************************************************************* */
class Skybox {
public:
	//Varaibles
	vector<VertexSkybox> vertices;
	vector<unsigned int> indices;
	GLuint cubeTexture;
	unsigned int VAO, VBO, EBO;

	/* *********************************************************************************************************
	Standard names for skybox textures:
	FRONT	negz
	BACK	posz
	TOP		posy
	BOTTOM	negy
	LEFT	negx
	RIGHT	posx
	********************************************************************************************************* */
	void createSkybox(
		const char* negz,
		const char* posz,
		const char* posy,
		const char* negy,
		const char* negx,
		const char* posx) {

		//Create VBO
		createVBO();

		//// generate a cube-map texture to hold all the sides
		glActiveTexture(GL_TEXTURE0);
		glGenTextures(1, &this->cubeTexture);

		// load each image and copy into a side of the cube-map texture
		load_cube_map_side(*&this->cubeTexture, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, negz);
		load_cube_map_side(*&this->cubeTexture, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, posz);
		load_cube_map_side(*&this->cubeTexture, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, posy);
		load_cube_map_side(*&this->cubeTexture, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, negy);
		load_cube_map_side(*&this->cubeTexture, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, negx);
		load_cube_map_side(*&this->cubeTexture, GL_TEXTURE_CUBE_MAP_POSITIVE_X, posx);

		//// format cube map texture
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}

	void Draw(Shader shader)
	{
		glDepthMask(GL_FALSE);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, this->cubeTexture);

		glBindVertexArray(VAO);	
		glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);

		//always good practice to set everything back to defaults once configured.
		glDepthMask(GL_TRUE);
	}

private:
	void createVBO() {
		this->vertices = {
			VertexSkybox(glm::vec3(-1.0,  -1.0,  -1.0)),
			VertexSkybox(glm::vec3(-1.0,  -1.0,  1.0)),
			VertexSkybox(glm::vec3(-1.0,  1.0,  -1.0)),
			VertexSkybox(glm::vec3(-1.0,  1.0,  1.0)),
			VertexSkybox(glm::vec3(1.0,  -1.0,  -1.0)),
			VertexSkybox(glm::vec3(1.0,  -1.0,  1.0)),
			VertexSkybox(glm::vec3(1.0,  1.0,  -1.0)),
			VertexSkybox(glm::vec3(1.0,  1.0,  1.0))
		};

		for (int i = 0; i < vertices.size(); i++) {
			this->vertices[i].Position *= 10.0f;
		}

		this->indices = { 1, 7, 5,
			1, 3, 7,
			1, 4, 3,
			1, 2, 4,
			3, 8, 7,
			3, 4, 8,
			5, 7, 8,
			5, 8, 6,
			1, 5, 6,
			1, 6, 2,
			2, 6, 8,
			2, 8, 4 };

		for (int i = 0; i < indices.size(); i++) {
			this->indices[i] -= 1;
		}

		// create buffers/arrays
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		glGenBuffers(1, &EBO);

		glBindVertexArray(VAO);
		// load data into vertex buffers
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		// A great thing about structs is that their memory layout is sequential for all its items.
		// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
		// again translates to 3/2 floats which translates to a byte array.
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(VertexSkybox), &vertices[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

		// set the vertex attribute pointers
		// vertex Positions
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexSkybox), (void*)0);
		glBindVertexArray(0);
	}

	bool load_cube_map_side(
		GLuint texture, GLenum side_target, const char* file_name) {
		glBindTexture(GL_TEXTURE_CUBE_MAP, texture);

		int x, y, n;
		int force_channels = 4;
		unsigned char*  image_data = stbi_load(
			file_name, &x, &y, &n, force_channels);
		if (!image_data) {
			fprintf(stderr, "ERROR: could not load %s\n", file_name);
			return false;
		}
		// non-power-of-2 dimensions check
		if ((x & (x - 1)) != 0 || (y & (y - 1)) != 0) {
			fprintf(stderr,
				"WARNING: image %s is not power-of-2 dimensions\n",
				file_name);
		}

		// copy image data into 'target' side of cube map
		glTexImage2D(
			side_target,
			0,
			GL_RGBA,
			x,
			y,
			0,
			GL_RGBA,
			GL_UNSIGNED_BYTE,
			image_data);
		free(image_data);
		return true;
	}
};
#endif