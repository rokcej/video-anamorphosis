// OpenGL shaders and functions for compiling them

#pragma once

#include <GL/glew.h>

const char* vsSource = "\n\
#version 450 core \n\
layout (location = 0) in vec3 aPos; \n\
layout (location = 1) in vec3 aColor; \n\
out vec3 vColor; \n\
uniform mat4 uPVM; \n\
void main() { \n\
	gl_Position = uPVM * vec4(aPos, 1.0f); \n\
	vColor = aColor; \n\
} \n\
";

const char* fsSource = "\n\
#version 450 core \n\
in vec3 vColor; \n\
out vec4 oColor; \n\
void main() { \n\
	oColor = vec4(vColor, 1.0f); \n\
} \n\
";


const char* vsSourceTex = "\n\
#version 450 core \n\
layout (location = 0) in vec3 aPos; \n\
layout (location = 1) in vec2 aTexCoord; \n\
out vec2 vTexCoord; \n\
void main() { \n\
	gl_Position = vec4(aPos, 1.0f); \n\
	vTexCoord = aTexCoord; \n\
} \n\
";

const char* fsSourceTex = "\n\
#version 450 core \n\
in vec2 vTexCoord; \n\
out vec4 oColor; \n\
uniform sampler2D uTex; \n\
void main() { \n\
	oColor = texture(uTex, vTexCoord); \n\
} \n\
";

GLuint compileProgram(const char *vsSrc, const char *fsSrc) {
	// Error log
	GLint status;
	char buf[1024];

	// Vertex shader
	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vs, 1, &vsSrc, NULL);
	glCompileShader(vs);
	glGetShaderiv(vs, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE) {
		glGetShaderInfoLog(vs, sizeof(buf), NULL, buf);
		std::cout << "[Vertex shader compile error log]" << std::endl << buf << std::endl;
	}

	// Fragment shader
	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fs, 1, &fsSrc, NULL);
	glCompileShader(fs);
	glGetShaderiv(fs, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE) {
		glGetShaderInfoLog(fs, sizeof(buf), NULL, buf);
		std::cout << "[Fragment shader compile error log]" << std::endl << buf << std::endl;
	}

	// Program
	GLuint prog = glCreateProgram();
	glAttachShader(prog, vs);
	glAttachShader(prog, fs);
	glLinkProgram(prog);
	glGetProgramiv(prog, GL_LINK_STATUS, &status);
	if (status == GL_FALSE) {
		glGetProgramInfoLog(prog, sizeof(buf), NULL, buf);
		std::cout << "[Program link error log]" << std::endl << buf << std::endl;
	}
	glValidateProgram(prog);
	glGetProgramiv(prog, GL_VALIDATE_STATUS, &status);
	if (status == GL_FALSE) {
		glGetProgramInfoLog(prog, sizeof(buf), NULL, buf);
		std::cout << "[Program validate error log]" << std::endl << buf << std::endl;
	}

	// Cleanup
	glDeleteShader(vs);
	glDeleteShader(fs);

	return prog;
}
