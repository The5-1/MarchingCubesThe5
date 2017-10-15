#ifndef TEXTURE_H
#define TEXTURE_H

#include <GL/glew.h>
#include <GL/glut.h>
#include <iostream>

class Texture {
public:
	int w, h;
	GLuint index;

public:

	Texture() {
	}

	Texture(int w, int h, int internal_format, int format, int type){

		this->w = w;
		this->h = h;

		glGenTextures(1, &index);
		//std::cout << "tex id " << index << std::endl;

		// "Bind" the newly created texture : all future texture functions will modify this texture
		glBindTexture(GL_TEXTURE_2D, index);


		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

		// Give an empty image to OpenGL ( the last "0" )
		glTexImage2D(GL_TEXTURE_2D, 0, internal_format, w, h, 0, format, type, 0);
	}

	void Bind() {
		glBindTexture(GL_TEXTURE_2D, index);
	}

	int Unbind() {
		glBindTexture(GL_TEXTURE_2D, 0);
		return index;
	}

	GLuint Index() const { 
		return index; 
	}

};

#endif
