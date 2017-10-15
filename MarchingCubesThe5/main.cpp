#define GLEW_STATIC //Using the static lib, so we need to enable it
#include <iostream>
#include <GL/glew.h>
#include <GL/glut.h>
#include <Ant/AntTweakBar.h>
#include <memory>
#include "helper.h"
#include "Shader.h"
#include "Model.h"
#include "Skybox.h"
#include "times.h"
#include "InstancedMesh.h"
#include "FBO.h"
#include "Texture.h"
#include "glm/gtx/string_cast.hpp"
#include "marchingCubesVolume.h"

/*
Marching cube dimensions!!!
*/
const int marchingCubesDimension = 10;


//Time
Timer timer;
int frame;
long timeCounter, timebase;
char timeString[50];

//Resolution (has to be changed in helper.h too)
glm::vec2 resolution = glm::vec2(1024, 768);

//Externals
cameraSystem cam(1.0f, 1.0f, glm::vec3(20.95f, 20.95f, -0.6f));
glm::mat4 projMatrix;
glm::mat4 viewMatrix;

Mesh boxMesh;

//Skybox
Skybox skybox;
char* negz = "C:/Dev/Assets/SkyboxTextures/Yokohama2/negz.jpg";
char* posz = "C:/Dev/Assets/SkyboxTextures/Yokohama2/posz.jpg";
char* posy = "C:/Dev/Assets/SkyboxTextures/Yokohama2/posy.jpg";
char* negy = "C:/Dev/Assets/SkyboxTextures/Yokohama2/negy.jpg";
char* negx = "C:/Dev/Assets/SkyboxTextures/Yokohama2/negx.jpg";
char* posx = "C:/Dev/Assets/SkyboxTextures/Yokohama2/posx.jpg";

//Shaders
Shader basicShader;
Shader modelLoaderShader;

//Skybox
Shader skyboxShader;

//Models
Model sponzaModel;
simpleModel *teaPot = 0;

//Marching Cubes
marchingCubesVolume *mVolume;

// tweak bar
TwBar *tweakBar;
bool wireFrameTeapot = false;
bool wireFrameImplicit = false;
bool backfaceCull = false;
bool showGrid = false;
float resizeMarchingCubes = 1.0f;
/* *********************************************************************************************************
TweakBar
********************************************************************************************************* */
void setupTweakBar() {
	TwInit(TW_OPENGL_CORE, NULL);
	tweakBar = TwNewBar("Settings");
	TwAddSeparator(tweakBar, "Wireframe", nullptr);
	TwAddVarRW(tweakBar, "Wireframe Implicit", TW_TYPE_BOOLCPP, &wireFrameImplicit, " label='Wireframe Implicit' ");
	TwAddVarRW(tweakBar, "Wireframe Teapot", TW_TYPE_BOOLCPP, &wireFrameTeapot, " label='Wireframe Teapot' ");

	TwAddSeparator(tweakBar, "Backface Culling", nullptr);
	TwAddVarRW(tweakBar, "Backface Cull", TW_TYPE_BOOLCPP, &backfaceCull, " label='Backface Cull' ");

	TwAddSeparator(tweakBar, "Resize Cube", nullptr);
	TwAddVarRW(tweakBar, "Cube size", TW_TYPE_FLOAT, &resizeMarchingCubes, " label='Cube size' min=1.0 step=0.1 max=10.0 ");
	TwAddVarRW(tweakBar, "Cube grid", TW_TYPE_BOOLCPP, &showGrid, " label='Cube grid' ");

}

/* *********************************************************************************************************
Initiation
********************************************************************************************************* */
GLuint vboBox[2];
vector<glm::vec3> boxVertices;
vector<unsigned int> boxIndices;
void init() {
	/*****************************************************************
	Draw a box with GL_QUADS
	-tutorial: http://www.songho.ca/opengl/gl_vertexarray_quad.html
	-vboBox, boxIndices, boxVertices are a global variable
	*****************************************************************/

	boxVertices = { glm::vec3(1.0, 0.0, 1.0),
		glm::vec3(0.0, 0.0, 1.0),
		glm::vec3(0.0, 0.0, 0.0),
		glm::vec3(1.0, 0.0, 0.0),
		glm::vec3(1.0, 1.0, 0.0),
		glm::vec3(1.0, 1.0, 1.0),
		glm::vec3(0.0, 1.0, 1.0),
		glm::vec3(0.0, 1.0, 0.0)
		 };

	boxIndices = { 0,1,2,3,
					0,3,4,5,
					0,5,6,1,
					1,6,7,2,
					7,4,3,2,
					4,7,6,5 };


	glGenBuffers(2, vboBox);
	glBindBuffer(GL_ARRAY_BUFFER, vboBox[0]);
	glBufferData(GL_ARRAY_BUFFER, boxVertices.size() * sizeof(float) * 3, boxVertices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboBox[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, boxIndices.size() * sizeof(unsigned int), boxIndices.data(), GL_STATIC_DRAW);


	/*****************************************************************
	Marching-Cube Box for visual representation 
	*****************************************************************/
	//boxMesh.createBox();

	/*
	Teapot-Mesh (Uses resize to unit-cube!!!), alternative use a pointcloud
	*/
	teaPot = new simpleModel("C:/Dev/Assets/Teapot/teapot.obj");
	teaPot->resizeToUniformCube();
	teaPot->upload();

	/*
	Marching-Cubes-Algorithm
	*/

	mVolume =  new marchingCubesVolume(marchingCubesDimension, marchingCubesDimension, marchingCubesDimension);
	mVolume->computeVolumeForImplicitFunction(teaPot->vertices, teaPot->normals);
	//mVolume->computeVolumeForWeightedLSQ(teaPot->vertices, teaPot->normals, 0.01f);
	mVolume->calculateNewVertices();
	mVolume->saveVertices();
	mVolume->upload();

	/*
	mVolume->buildKDtree(teaPot->vertices);
	int id[1];
	mVolume->findkClosestPoints(glm::vec3(-0.428530f, 0.912498f, -0.029883f), 1, id);
	std::cout << id[1] << std::endl;
	*/


	/*****************************************************************
	Skybox (Only for aesthetic reasons, can be deleted)
	*****************************************************************/
	skybox.createSkybox(negz, posz, posy, negy, negx, posx);
}


void loadShader(bool init) {
	basicShader = Shader("./shader/standard.vs.glsl", "./shader/standard.fs.glsl");
	modelLoaderShader = Shader("./shader/modelLoader.vs.glsl", "./shader/modelLoader.fs.glsl");
	skyboxShader = Shader("./shader/skybox.vs.glsl", "./shader/skybox.fs.glsl");

}

/* *********************************************************************************************************
Scenes: Unit cube + Pointcloud, Results of marching cubes
********************************************************************************************************* */
void sponzaStandardScene(){
	skyboxShader.enable();
	skyboxShader.uniform("projMatrix", projMatrix);
	skyboxShader.uniform("viewMatrix", cam.cameraRotation);
	skybox.Draw(skyboxShader);
	skyboxShader.disable();

	if (backfaceCull) {
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
	}


	glm::mat4 modelMatrix;

	/*
	Step 1: Draw Pointcloud/Mesh into a cube, so we can see if mesh is completely inside the unit cube
	*/
	//Draw Cube
	basicShader.enable();
	basicShader.uniform("projMatrix", projMatrix);
	basicShader.uniform("viewMatrix", viewMatrix);

	glPolygonMode(GL_FRONT, GL_LINE);
	glPolygonMode(GL_BACK, GL_LINE);

	float anti_Z_fighting = 0.01;
	modelMatrix = glm::mat4(1.0);
	modelMatrix = glm::scale(modelMatrix, glm::vec3(resizeMarchingCubes + anti_Z_fighting));
	basicShader.uniform("modelMatrix", modelMatrix);
	basicShader.uniform("col", glm::vec3(1.0f, 1.0f, 1.0f));
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vboBox[0]);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboBox[1]);
	glDrawElements(GL_QUADS, boxIndices.size(), GL_UNSIGNED_INT, 0);
	
	if (showGrid) {
		float resize = 1.0f / float(marchingCubesDimension);
		for (int i = 0; i < marchingCubesDimension; i++) {
			for (int j = 0; j < marchingCubesDimension; j++) {
				for (int k = 0; k < marchingCubesDimension; k++) {
					modelMatrix = glm::mat4(1.0f);

					modelMatrix = glm::translate(modelMatrix, glm::vec3(float(i) * resizeMarchingCubes * resize, float(j) * resizeMarchingCubes * resize, float(k) * resizeMarchingCubes * resize));
					modelMatrix = glm::scale(modelMatrix, glm::vec3(resizeMarchingCubes * resize));

					basicShader.uniform("modelMatrix", modelMatrix);
					basicShader.uniform("col", glm::vec3(1.0f, 0.0f, 0.0f));
					glEnableVertexAttribArray(0);
					glBindBuffer(GL_ARRAY_BUFFER, vboBox[0]);
					glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
					glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboBox[1]);
					glDrawElements(GL_QUADS, boxIndices.size(), GL_UNSIGNED_INT, 0);
				}

			}

		}
	}
	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);
	basicShader.disable();

	//Draw Pointcloud/Mesh
	basicShader.enable();
	basicShader.uniform("projMatrix", projMatrix);
	basicShader.uniform("viewMatrix", viewMatrix);
	modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, 0.0f, 0.0f));
	modelMatrix = glm::scale(modelMatrix, glm::vec3(resizeMarchingCubes));
	basicShader.uniform("modelMatrix", modelMatrix);
	basicShader.uniform("col", glm::vec3(0.0f, 0.0f, 1.0f));
	if (wireFrameTeapot) {
		glPolygonMode(GL_FRONT, GL_LINE);
		glPolygonMode(GL_BACK, GL_LINE);
	}
	teaPot->draw();
	if (wireFrameTeapot) {
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);
	}
	basicShader.disable();

	/*
	Step 2: Draw results of marching cube
	*/
	basicShader.enable();
	basicShader.uniform("projMatrix", projMatrix);
	basicShader.uniform("viewMatrix", viewMatrix);
	modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(5.0f, 5.0f, 5.0f));
	modelMatrix = glm::scale(modelMatrix, glm::vec3(5.0f));
	basicShader.uniform("modelMatrix", modelMatrix);
	basicShader.uniform("col", glm::vec3(0.0f, 1.0f, 0.0f));
	if (wireFrameImplicit) {
		glPolygonMode(GL_FRONT, GL_LINE);
		glPolygonMode(GL_BACK, GL_LINE);
	}

	mVolume->draw();

	if (wireFrameImplicit) {
		glPolygonMode(GL_FRONT, GL_FILL);
		glPolygonMode(GL_BACK, GL_FILL);
	}
	basicShader.disable();
}	

/* *********************************************************************************************************
Display + Main
********************************************************************************************************* */
void display() {
	//Timer
	timer.update();
	//FPS-Counter
	frame++;
	timeCounter = glutGet(GLUT_ELAPSED_TIME);
	if (timeCounter - timebase > 1000) {
		sprintf_s(timeString, "FPS:%4.2f", frame*1000.0 / (timeCounter - timebase));
		timebase = timeCounter;
		frame = 0;
		glutSetWindowTitle(timeString);
	}

	//OpenGL Clears
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glClearColor(0.2f, 0.2f, 0.2f, 1);
	
	sponzaStandardScene();
			
	TwDraw(); //Draw Tweak-Bar

	glutSwapBuffers();
	glutPostRedisplay();

}

void myDelete() {
	delete mVolume;
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL);

	glutCreateWindow("Basic Framework");

	setupTweakBar();

	GLenum err = glewInit();
	if (GLEW_OK != err) {
		std::cerr << "Error : " << glewGetErrorString(err) << std::endl;
	}

	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutMotionFunc(onMouseMove);
	glutMouseFunc(onMouseDown);
	glutReshapeFunc(reshape);
	glutIdleFunc(onIdle);

	glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	TwGLUTModifiersFunc(glutGetModifiers);

	initGL();

	init();

	glutMainLoop();

	TwTerminate();
	myDelete();
	return 0;
}










