#pragma once
#include <glm\glm.hpp>
#include <vector>
#include <GL/glew.h>
#include <GL/glut.h>
//#include <ANN\ANN.h>

/*
Creates a uniform 3D-Volume
Requires the vertices of Scandata to be in the range of [0, 1]
http://www.diva-portal.org/smash/get/diva2:558597/FULLTEXT01.pdf
*/

/*
ToDo:
Change float to double precision?
*/

class marchingCubesVolume
{
//Variables
public:
	//Result if marching Cubes
	std::vector<glm::vec3> verticesResult;
	GLuint vbo;

	//Box 
	GLuint vboBox[2];
	std::vector<glm::vec3> boxVertices;
	std::vector<unsigned int> boxIndices;

	//kd-Tree
	//ANNkd_tree* m_kdTree; //! The ANN kD-Tree.
	//ANNpointArray m_dataPts;//! Points of the kd-tree.	
	//ANNpoint m_queryPt; //! Point structure for kd-tree queries

private:
	float* volValues;
	unsigned int xDim, yDim, zDim;
	float dx, dy, dz;

//Functions
public:
	//Constructor
	marchingCubesVolume();
	marchingCubesVolume(unsigned int _xDim, unsigned int _yDim, unsigned int _zDim);
	~marchingCubesVolume();

	

	//Utility
	void resetValues(float value);
	int getIndexVolume(unsigned int x, unsigned int y, unsigned int z);
	void setValue(unsigned int x, unsigned int y, unsigned int z, float value);
	float getValue(unsigned int x, unsigned int y, unsigned int z);
	glm::vec3 getPosition(unsigned int x, unsigned int y, unsigned int z);


	int findNearestNeighbour(glm::vec3 position, const std::vector<glm::vec3>& vertices);
	float evaluateHoppesImplicitFunction(glm::vec3 position, const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals);
	void computeVolumeForImplicitFunction(const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals);
	void calculateNewVertices();
	void upload();
	void draw();
	float evaluateWeightedLeastSquare(glm::vec3 position, const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals, float epsilon);
	void computeVolumeForWeightedLSQ(const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals, float epsilon);

	//void findkClosestPoints(const glm::vec3 & pt, int k, int * ids) const;
	//void buildKDtree(const std::vector<glm::vec3>& vertices);

	//Save Results
	void saveVertices();

private:
	void computeVolume();
};

