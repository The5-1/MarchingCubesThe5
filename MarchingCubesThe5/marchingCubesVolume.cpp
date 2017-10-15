#include "marchingCubesVolume.h"
#include "marchingCubes.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>


marchingCubesVolume::marchingCubesVolume()
{
}

marchingCubesVolume::marchingCubesVolume(unsigned int _xDim, unsigned int _yDim, unsigned int _zDim)
{
	this->xDim = _xDim;
	this->yDim = _yDim;
	this->zDim = _zDim;

	volValues = new float[_xDim * _yDim * _zDim];
	this->computeVolume();
}

marchingCubesVolume::~marchingCubesVolume()
{
	delete[] this->volValues;

	//if (m_kdTree != NULL) delete m_kdTree;
	//if (m_dataPts != NULL) annDeallocPts(m_dataPts);
	//annDeallocPt(m_queryPt);
	//annClose();
}

void marchingCubesVolume::computeVolume()
{
	this->dx = 1.0f / (this->xDim - 1);
	this->dy = 1.0f / (this->yDim - 1);
	this->dz = 1.0f / (this->zDim - 1);
}

//Utility Functions

void marchingCubesVolume::resetValues(float value = 0.0)
{
	for (int i = 0; i < this->xDim * this->yDim * this->zDim; i++) {
		this->volValues[i] = value;
	}
}

int marchingCubesVolume::getIndexVolume(unsigned int x, unsigned int y, unsigned int z) {
	return x * this->yDim * this->zDim + y * this->zDim + z;
}

void marchingCubesVolume::setValue(unsigned int x, unsigned int y, unsigned int z, float value)
{
	int position = getIndexVolume(x, y, z);
	this->volValues[position] = value;
}

float marchingCubesVolume::getValue(unsigned int x, unsigned int y, unsigned int z)
{
	int position = getIndexVolume(x, y, z);
	return this->volValues[position];
}

glm::vec3 marchingCubesVolume::getPosition(unsigned int x, unsigned int y, unsigned int z) {
	return glm::vec3(float(x) * this->dx, float(y) * this->dy, float(z) * this->dz);
}


//Marching Cubes Algorithm
int marchingCubesVolume::findNearestNeighbour(glm::vec3 position, const std::vector<glm::vec3>& vertices) {
	int index = 0;
	float dist = glm::distance(vertices[0], position);

	for (int i = 1; i < vertices.size(); i++) {
		float tempDist = glm::distance(vertices[i], position);
		if (tempDist < dist) {
			dist = tempDist;
			index = i;
		}
	}
	return index;
}

float marchingCubesVolume::evaluateHoppesImplicitFunction(glm::vec3 position, const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals) {
	//we passed the center of the grid-cell ("position")
	//find the vertex closest to the cells center and get its pos and normal
	int indexNN = this->findNearestNeighbour(position, vertices);
	glm::vec3 vertex = vertices[indexNN];
	glm::vec3 normal = normals[indexNN];

	//now just take the dot-product of the vector from the vertex to the center and its normal
	//--> signed field, the distance is the length that results from the dot product
	//--> marching cubes then finds the ZERO between adjacent cells: if both are of different sign there is a zero intersection!
	return glm::dot(position - vertex, normal);
}

void marchingCubesVolume::computeVolumeForImplicitFunction(const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals)
{
	for (unsigned int x = 0; x < this->xDim; x++){
		for (unsigned int y = 0; y < this->yDim; y++){
			for (unsigned int z = 0; z < this->zDim; z++){
				glm::vec3 pos_in_space = this->getPosition(x, y, z);

				float val = this->evaluateHoppesImplicitFunction(pos_in_space, vertices, normals);
				this->setValue(x, y, z, val);
			}
		}
	}
}

void marchingCubesVolume::calculateNewVertices() {
	for (unsigned int x = 0; x < xDim - 1; x++){
		for (unsigned int y = 0; y < yDim - 1; y++){
			for (unsigned int z = 0; z < zDim - 1; z++){
				processVolumeCell(this, x, y, z, 0.00f);
			}
		}
	}
}

void marchingCubesVolume::upload() {
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, this->verticesResult.size() * sizeof(float) * 3, this->verticesResult.data(), GL_STATIC_DRAW);
}

void marchingCubesVolume::draw() {
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glDrawArrays(GL_TRIANGLES, 0, verticesResult.size());
	glDisableVertexAttribArray(0);
}

/*
Interpolating and Approximating Implicit Surfaces from Polygon Soup
*/
/*
http://kucg.korea.ac.kr/new/research/Geometry/MLS/
http://cs.nyu.edu/~panozzo/gp/03%20-%20Reconstruction.pdf
*/
float marchingCubesVolume::evaluateWeightedLeastSquare(glm::vec3 position, const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals, float epsilon) {
	
	//This happens for EACH point of our grid! (Needs to be optimized)
	//A(x,y,z)*c = f(x,yz)
	//a*x + b*y + c*z + d = f(x,y,z) is the implicit function we search
	//c = coeficients (a,b,c,d)

	int verticesSize = vertices.size();
	Eigen::MatrixXf vertexMatrix(verticesSize, 4); //= Matrix A with (x,y,z,1)
	Eigen::MatrixXf weightMatrix(verticesSize, 4); //= Diagonal Matrix W with weights w_i for each f_i(x,y,z)
	Eigen::VectorXf weightVector(verticesSize); //= temporary vector we convert to the diagonal matrix later
	Eigen::VectorXf resultVector(verticesSize); //= given target values for f(x,y,z)

	Eigen::VectorXf coefficientsVectorForImplicitFunction(4); //the coefficent vector c with the (a,b,c,d) we search

	for (int i = 0; i < verticesSize; i++) {
		//Fill Vertex Matirx A (for the current grid point)
		//each row with (x,y,z,1)
		vertexMatrix(i, 0) = vertices[i].x;
		vertexMatrix(i, 1) = vertices[i].y;
		vertexMatrix(i, 2) = vertices[i].z;
		vertexMatrix(i, 3) = 1.0f;

		//Weights
		//Square Euclidean distance from the vertex to the current grid point 
		float squaredLength = (position.x - vertices[i].x)*(position.x - vertices[i].x) + (position.y - vertices[i].y)*(position.y - vertices[i].y) + (position.z - vertices[i].z)*(position.z - vertices[i].z);
		weightVector(i) = 1.0f/(squaredLength + epsilon * epsilon);

		//Result Vector
		//dot product from vert-to-pos and normal
		//if we were to just search for f(x,y,z) = 0, we would set all coefficients = 0
		//we take the hoppes term to give some guidance for our functions (are we inside or outside?)
		resultVector[i] = glm::dot(position - vertices[i], normals[i]);
	}

	weightMatrix = weightVector.asDiagonal();


	//to move all the matrices to one side of the equation we need the pseudo inverse (for our NxM matrix)
	//We have: A * c = b
	//We want: c = A^-1 * b
	//Both sides: * A^T
	//A^T * A results in NxN  matrix that can be inverted and move to the other side!
	//so the Pseudoinverse is: (A^T * A^)^-1 * A^T
	coefficientsVectorForImplicitFunction = (vertexMatrix.transpose() * weightMatrix * weightMatrix * vertexMatrix).inverse() * vertexMatrix.transpose() * weightMatrix * weightMatrix * resultVector;


	//lastly we evaluate our implicit function for the observed grid-point
	return coefficientsVectorForImplicitFunction[0] * position.x + coefficientsVectorForImplicitFunction[1] * position.y + coefficientsVectorForImplicitFunction[2] * position.z + coefficientsVectorForImplicitFunction[3];
}

void marchingCubesVolume::computeVolumeForWeightedLSQ(const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals, float epsilon)
{
	for (unsigned int x = 0; x < this->xDim; x++) {
		for (unsigned int y = 0; y < this->yDim; y++) {
			for (unsigned int z = 0; z < this->zDim; z++) {
				glm::vec3 pos_in_space = this->getPosition(x, y, z);

				float val = this->evaluateWeightedLeastSquare(pos_in_space, vertices, normals, epsilon);
				this->setValue(x, y, z, val);
			}
		}
	}
}

//inline void marchingCubesVolume::findkClosestPoints(const glm::vec3& pt, int k, int* ids) const
//{
//	for (unsigned int c = 0; c < 3; ++c)
//	{
//		m_queryPt[c] = pt[c];
//	}
//
//	//! Distance value from kd-tree queries
//	ANNdist* dist = new ANNdist[k];
//
//	m_kdTree->annkSearch(m_queryPt, k, ids, dist, 0);
//
//	delete[] dist;
//}
//
///*
//Build kd-Tree to speed up nearest neibhour-search
//*/
//void marchingCubesVolume::buildKDtree(const std::vector<glm::vec3>& vertices) {
//	unsigned int m_numPts = vertices.size();
//
//	m_dataPts = annAllocPts(m_numPts, 3);
//	m_queryPt = annAllocPt(3);
//
//	// Copy data into ANN array
//	for (unsigned int i1 = 0; i1 < m_numPts; i1++){
//
//		const glm::vec3& pt = vertices[i1];
//
//		for (unsigned int c = 0; c < 3; ++c){
//			m_dataPts[i1][c] = pt[c];
//		}
//
//	}
//	// Build kD-Tree.
//	m_kdTree = new ANNkd_tree(m_dataPts, (int)m_numPts, 3);
//}



/*
Save results
*/
void marchingCubesVolume::saveVertices()
{
	std::ofstream file;
	file.open("resultVertices.txt");
	file << "* Result Vertices of marching cube \n";
	file << "* Vertices (3 floats) are saved per line. Every three lines are a triangle.\n";
	file << "* Author: The5_2 \n";
	file << "* \n";
	file << "* Vertices: "<< verticesResult.size()<< " \n";
	for (int i = 0; i < this->verticesResult.size(); i++) {
		file << "v " << verticesResult[i].x << " " << verticesResult[i].y << " " << verticesResult[i].z << "\n";
	}
	file.close();
}
