#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <GL/glew.h>
#include <GL/glut.h>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define WIDTH 1024
#define HEIGHT 768

//https://blog.nobel-joergensen.com/2013/01/29/debugging-opengl-using-glgeterror/
//https://stackoverflow.com/a/3048361
#define gl_check_error(x) _gl_check_error(x,__FILE__,__LINE__,__FUNCTION__)

using namespace std;

//glut stuff
void keyboard(unsigned char key, int x, int y);

void onMouseMove(int x, int y);

void onMouseDown(int button, int state, int x, int y);

void onIdle();
void initGL();
void updateCamera();
void reshape(int w, int h);

void loadShader(bool init);

bool _gl_check_error(const char* arg, const char *file, int line, const char *function);

static string textFileRead(const char *fileName);

// Screen Space Quad
class simpleQuad
{
public:
	void upload();
	void draw();
protected:
	vector<unsigned int> indices;
	vector<glm::vec3> vertices;
	GLuint vbo[2];
};

// Simple Cube
class simpleCube
{
public:
	void upload();
	void draw();
protected:
	vector<unsigned int> indices;
	vector<glm::vec3> vertices;
	GLuint vbo[2];
};

// very simple geometry
class simpleModel
{
public:
	simpleModel();
	simpleModel(const char *fileName);
	~simpleModel();

	void resizeToUniformCube();

	void upload();
	void draw();
	void calculateNormals();
	glm::mat4 modelMatrix;
public:
	vector<unsigned int> indices;
	vector<glm::vec3> vertices;
	vector<glm::vec3> normals;
	GLuint vbo[3];
};

class groundPlane : public simpleModel
{
public:
	groundPlane(const float height, const float width);
};

class solidTorus : public simpleModel 
{
public:
	solidTorus(const float r, const float R, const float sides, const float rings);
};

class solidSphere
{
public:
	solidSphere(const float radius, const int slices, const int stacks);
	~solidSphere();
	void upload();
	void draw();
protected:
	vector<unsigned int> indices;
	vector<glm::vec2> uvs;
	vector<glm::vec3> vertices;
	GLuint vbo[3];
};

// image / texture
class Image
{
public:
	Image(const char *fileName);
	~Image();
	int makeTexture();
	unsigned int width;
	unsigned int height;
	//unsigned char* pixels;
	std::vector<unsigned char> image;
};


// timer

// camera sutff
class cameraSystem 
{
public:

	glm::vec4 position;   // position-vector
	glm::vec4 viewDir;    // viewing direction
	glm::vec4 upDir;      // up-vector
	glm::vec4 rightDir;   // right-vector (cross product of viewing- and up-direction)

	glm::mat4 cameraRotation;

	int currentX, currentY;
	float delta, mouseDelta;
	bool rightMouseDown;
	bool middleMouseDown;

	cameraSystem(float delta, float mouseDelta, glm::vec3 pos);

	void moveForward(float delta);
	void moveBackward(float delta);
	void moveUp(float delta);
	void moveDown(float delta);
	void moveRight(float delta);
	void moveLeft(float delta);
	void yaw(float angle);
	void pitch(float angle);
	void roll(float angle);

	void Update();

private:
	void updateCameraMatrix();
	void updateRotationMatrix(glm::mat4 view);

};

// 'random' sampler
static const float HALTON23[64][2] = {
	{0, -0.333333},
	{-0.5, 0.333333},
	{0.5, -0.777778},
	{-0.75, -0.111111},
	{0.25, 0.555556},
	{-0.25, -0.555556},
	{0.75, 0.111111},
	{-0.875, 0.777778},
	{0.125, -0.925926},
	{-0.375, -0.259259},
	{0.625, 0.407407},
	{-0.625, -0.703704},
	{0.375, -0.037037},
	{-0.125, 0.62963},
	{0.875, -0.481481},
	{-0.9375, 0.185185},
	{0.0625, 0.851852},
	{-0.4375, -0.851852},
	{0.5625, -0.185185},
	{-0.6875, 0.481481},
	{0.3125, -0.62963},
	{-0.1875, 0.037037},
	{0.8125, 0.703704},
	{-0.8125, -0.407407},
	{0.1875, 0.259259},
	{-0.3125, 0.925926},
	{0.6875, -0.975309},
	{-0.5625, -0.308642},
	{0.4375, 0.358025},
	{-0.0625, -0.753086},
	{0.9375, -0.0864198},
	{-0.96875, 0.580247},
	{0.03125, -0.530864},
	{-0.46875, 0.135802},
	{0.53125, 0.802469},
	{-0.71875, -0.901235},
	{0.28125, -0.234568},
	{-0.21875, 0.432099},
	{0.78125, -0.679012},
	{-0.84375, -0.0123457},
	{0.15625, 0.654321},
	{-0.34375, -0.45679},
	{0.65625, 0.209877},
	{-0.59375, 0.876543},
	{0.40625, -0.82716},
	{-0.09375, -0.160494},
	{0.90625, 0.506173},
	{-0.90625, -0.604938},
	{0.09375, 0.0617284},
	{-0.40625, 0.728395},
	{0.59375, -0.382716},
	{-0.65625, 0.283951},
	{0.34375, 0.950617},
	{-0.15625, -0.950617},
	{0.84375, -0.283951},
	{-0.78125, 0.382716},
	{0.21875, -0.728395},
	{-0.28125, -0.0617284},
	{0.71875, 0.604938},
	{-0.53125, -0.506173},
	{0.46875, 0.160494},
	{-0.03125, 0.82716},
	{0.96875, -0.876543},
	{-0.984375, -0.209877}
};
