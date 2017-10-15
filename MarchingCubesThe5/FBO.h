#ifndef FBO_H
#define FBO_H

#include <GL/glew.h>
#include <GL/glut.h>
#include "Texture.h"

#include <vector>
#include <map>
#include <string>

enum FBO_Type {
	FBO_EMPTY,
	FBO_RGB_DEPTH_16BIT,
	FBO_RGB_DEPTH_32BIT,
	FBO_RGBA_DEPTH_32BIT,
	FBO_GBUFFER_16BIT,
	FBO_GBUFFER_32BIT, 
	FBO_DEPTH_16BIT,
	FBO_DEPTH_32BIT};

class FBO {
public:
	string name;
	GLuint id;
	vector<string> attachment_name;
	vector<Texture> attachment_texture;
	//ToDo: Textures as a pair of name and index
	//std::map<string,Texture> attachment_texture;

	vector<GLuint> attachment_id;
	Texture *depthbuffer;
	enum { alterante_depthbuffer_id = 0 };
	GLuint next_att;
	int w, h;

public:
	FBO(const string &name, int w, int h, FBO_Type type)
		: name(name), w(w), h(h), next_att(0), depthbuffer(0) {
		glGenFramebuffers(1, &id);

		switch (type) {
			case FBO_EMPTY:
				break;

			case FBO_GBUFFER_32BIT: {
				Texture *diffuse = 0, *normal = 0, *position = 0, *depth = 0;
				//FixMe: Why does this crash the program

				glEnable(GL_TEXTURE_2D);
				gl_check_error("fbo");
				diffuse = new Texture(w, h, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("diffuse tex");
				normal = new Texture(w, h, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("normal tex");
				position = new Texture(w, h, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("position tex");
				depth = new Texture(w, h, GL_DEPTH_COMPONENT32, GL_DEPTH_COMPONENT, GL_FLOAT);
				gl_check_error("depth tex");

				//FixMe: This alone works
				this->Bind();
				this->AddTextureAsColorbuffer("diffuse", diffuse);
				this->AddTextureAsColorbuffer("normal", normal);
				this->AddTextureAsColorbuffer("position", position);
				this->AddTextureAsDepthbuffer(depth);
				this->Check();
				this->Unbind();
				gl_check_error("post fbo setup");
				break;
			}
			case FBO_GBUFFER_16BIT: {
				Texture *diffuse = 0, *normal = 0, *position = 0, *depth = 0;
				//FixMe: Why does this crash the program

				glEnable(GL_TEXTURE_2D);
				gl_check_error("fbo");
				diffuse = new Texture(w, h, GL_RGBA16F, GL_RGBA, GL_FLOAT);	gl_check_error("diffuse tex");
				normal = new Texture(w, h, GL_RGBA16F, GL_RGBA, GL_FLOAT);	gl_check_error("normal tex");
				position = new Texture(w, h, GL_RGBA16F, GL_RGBA, GL_FLOAT);	gl_check_error("position tex");
				depth = new Texture(w, h, GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT, GL_FLOAT);
				gl_check_error("depth tex");

				//FixMe: This alone works
				this->Bind();
				this->AddTextureAsColorbuffer("diffuse", diffuse);
				this->AddTextureAsColorbuffer("normal", normal);
				this->AddTextureAsColorbuffer("position", position);
				this->AddTextureAsDepthbuffer(depth);
				this->Check();
				this->Unbind();
				gl_check_error("post fbo setup");
				break;
			}
			case FBO_RGB_DEPTH_16BIT: {
				Texture *rgb = 0, *depth = 0;
				//FixMe: Why does this crash the program

				glEnable(GL_TEXTURE_2D);
				gl_check_error("fbo");
				rgb = new Texture(w, h, GL_RGB16F, GL_RGB, GL_FLOAT);	gl_check_error("rgb tex");
				depth = new Texture(w, h, GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT, GL_FLOAT);
				gl_check_error("depth tex");

				//FixMe: This alone works
				this->Bind();
				this->AddTextureAsColorbuffer("rgb", rgb);
				this->AddTextureAsDepthbuffer(depth);
				this->Check();
				this->Unbind();
				gl_check_error("post fbo setup");
				break;
			}
			case FBO_RGB_DEPTH_32BIT: {
				Texture *rgb = 0, *depth = 0;
				//FixMe: Why does this crash the program

				glEnable(GL_TEXTURE_2D);
				gl_check_error("fbo");
				rgb = new Texture(w, h, GL_RGB32F, GL_RGB, GL_FLOAT);	gl_check_error("rgb tex");
				depth = new Texture(w, h, GL_DEPTH_COMPONENT32, GL_DEPTH_COMPONENT, GL_FLOAT);
				gl_check_error("depth tex");

				//FixMe: This alone works
				this->Bind();
				this->AddTextureAsColorbuffer("rgb", rgb);
				this->AddTextureAsDepthbuffer(depth);
				this->Check();
				this->Unbind();
				gl_check_error("post fbo setup");
				break;
			}
			case FBO_RGBA_DEPTH_32BIT: {
				Texture *rgba = 0, *depth = 0;
				//FixMe: Why does this crash the program

				glEnable(GL_TEXTURE_2D);
				gl_check_error("fbo");
				rgba = new Texture(w, h, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("rgb tex");
				depth = new Texture(w, h, GL_DEPTH_COMPONENT32, GL_DEPTH_COMPONENT, GL_FLOAT);
				gl_check_error("depth tex");

				//FixMe: This alone works
				this->Bind();
				this->AddTextureAsColorbuffer("rgba", rgba);
				this->AddTextureAsDepthbuffer(depth);
				this->Check();
				this->Unbind();
				gl_check_error("post fbo setup");
				break;
			}
			case FBO_DEPTH_16BIT: {
				// We won't use a color texture for a only depth FBO
				Texture *depth = 0;
				glEnable(GL_TEXTURE_2D);
				depth = new Texture(w, h, GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT, GL_FLOAT);
				this->Bind();
				glDrawBuffer(GL_NONE);
				glReadBuffer(GL_NONE);
				this->AddTextureAsDepthbuffer(depth);
				this->Check();
				this->Unbind();
				gl_check_error("post fbo setup");
				break;
			}
			case FBO_DEPTH_32BIT: {
				// We won't use a color texture for a only depth FBO
				Texture *depth = 0;
				glEnable(GL_TEXTURE_2D);
				depth = new Texture(w, h, GL_DEPTH_COMPONENT32, GL_DEPTH_COMPONENT, GL_FLOAT);
				this->Bind();
				glDrawBuffer(GL_NONE);
				glReadBuffer(GL_NONE);
				this->AddTextureAsDepthbuffer(depth);
				this->Check();
				this->Unbind();
				gl_check_error("post fbo setup");
				break;
			}
		}

	}

	void Bind() {
		//glBindTexture(GL_TEXTURE_2D, 0); //Just to make sure the texture is not bound
		gl_check_error("pre bind");
		glBindFramebuffer(GL_FRAMEBUFFER, id);
		gl_check_error("in bind");
		if (depthbuffer == 0)
			glBindRenderbuffer(GL_RENDERBUFFER, alterante_depthbuffer_id);
		gl_check_error("in bind");
		if (next_att != 0)
			glDrawBuffers(next_att, &attachment_id[0]); //in which buffers of the fbo do we draw
		else
			glDrawBuffer(GL_NONE);
		gl_check_error("post bind");

		//glViewport(0, 0, FBOwidth, FBOheight) //We should change the resoultion to the resolution of our FBO
	}

	void AddTextureAsColorbuffer(const string &name, const Texture *img) {
		attachment_name.push_back(name);
		attachment_texture.push_back(*img);
		attachment_id.push_back(GL_COLOR_ATTACHMENT0 + next_att);
		glFramebufferTexture2D(GL_FRAMEBUFFER, attachment_id[next_att], GL_TEXTURE_2D, img->Index(), 0);
		++next_att;
		gl_check_error(("Error binding texture " + name + " to fbo " + this->name).c_str());
	}

	void AddTextureAsDepthbuffer(Texture *img) {
		//Depth-Buffer can also be saved as non Textures: https://www.youtube.com/watch?v=21UsMuFTN0k
		depthbuffer = img;
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, img->Index(), 0);
	}

	void Unbind() { //Switch back to default framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);
	}

	void Check() {
		int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (status != GL_FRAMEBUFFER_COMPLETE)
			cerr << "Framebuffer " << name << " is not complete: " << status << endl;
	}

	void fillFBO(int width, int height)
	{
		Texture *diffuse = 0, *normal = 0, *position = 0, *depth = 0;
		//FixMe: Why does this crash the program

		glEnable(GL_TEXTURE_2D);
		gl_check_error("fbo");
		diffuse = new Texture(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("diffuse tex");
		normal = new Texture(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("normal tex");
		position = new Texture(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);	gl_check_error("position tex");
		depth = new Texture(width, height, GL_DEPTH_COMPONENT32, GL_DEPTH_COMPONENT, GL_FLOAT);
		gl_check_error("depth tex");

		//FixMe: This alone works
		this->Bind();
		this->AddTextureAsColorbuffer("diffuse", diffuse);
		this->AddTextureAsColorbuffer("normal", normal);
		this->AddTextureAsColorbuffer("position", position);
		this->AddTextureAsDepthbuffer(depth);
		this->Check();
		this->Unbind();
		gl_check_error("post fbo setup");
	}

	//Bind all Textures
	void bindAllTextures() {
		for (int i = 0; i < attachment_texture.size(); i++) {
			glActiveTexture(GL_TEXTURE0 + i);
			attachment_texture[i].Bind();
		}

		glActiveTexture(GL_TEXTURE0 + attachment_texture.size());
		depthbuffer->Bind();
	}

	void unbindAllTextures() {
		for (int i = 0; i < attachment_texture.size(); i++) {
			attachment_texture[i].Unbind();
		}

		glActiveTexture(GL_TEXTURE0 + attachment_texture.size());
		depthbuffer->Unbind();
	}

	//Bind single Texture
	void bindTexture(unsigned int textureNumber, GLuint activeTexture = 0) {
		if (textureNumber >= attachment_texture.size()) {
			std::cout << "ERROR bindTexture: Index out of bound." << std::endl;
			return;
		}
		glActiveTexture(GL_TEXTURE0 + activeTexture);
		attachment_texture[textureNumber].Bind();
	}

	void unbindTexture(unsigned int textureNumber, GLuint activeTexture = 0) {
		if (textureNumber >= attachment_texture.size()) {
			std::cout << "ERROR bindTexture: Index out of bound." << std::endl;
			return;
		}
		glActiveTexture(GL_TEXTURE0 + activeTexture);
		attachment_texture[textureNumber].Unbind();
	}

	//Bind depth
	void bindDepth(GLuint activeTexture = 0) {
		glActiveTexture(GL_TEXTURE0 + activeTexture);
		depthbuffer->Bind();
	}

	void unbindDepth(GLuint activeTexture = 0) {
		glActiveTexture(GL_TEXTURE0 + activeTexture);
		depthbuffer->Unbind();
	}
};

#endif
