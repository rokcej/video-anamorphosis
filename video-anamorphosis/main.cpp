#define _CRT_SECURE_NO_WARNINGS
// Kinect
#include <Windows.h>
#include <Ole2.h>
#include <Kinect.h>
// Stdlib
#include <iostream>
#include <fstream>
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <vector>
#include <thread>
// GLEW
#include <GL/glew.h>
// GLFW
#include <GLFW/glfw3.h>
// STB_Image
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
// My libs
#include "shaders.h"
#include "bmp.h"

#define WIDTH 1600 // Window res
#define HEIGHT 900
#define DWIDTH 512 // Depth sensor res
#define DHEIGHT 424
#define CWIDTH 1920 // Color camera res
#define CHEIGHT 1080

#define SAFE_RELEASE(ptr) { if (ptr) { (ptr)->Release(); (ptr) = nullptr; } }

// GLFW variables
GLFWwindow *window = nullptr;

// OpenGL variables
GLuint prog; // Shader program
GLuint vao, vboDepth, vboColor;
glm::mat4 viewMat, projMat, pvmMat;

// Kinect variables
IKinectSensor *sensor = nullptr; // Kinect sensor
IMultiSourceFrameReader *reader = nullptr; // Kinect depth data reader
ICoordinateMapper *mapper = nullptr;

uint8_t rgbaBuf[CWIDTH * CHEIGHT * 4];
ColorSpacePoint depth2rgb[DWIDTH * DHEIGHT];
CameraSpacePoint depth2xyz[DWIDTH * DHEIGHT];

// Interaction
float radius = 1.5f;
float angle = 0.0f;
float speed = 0.005f;

bool initKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor)))
		return false;

	if (!sensor)
		return false;

	sensor->get_CoordinateMapper(&mapper);
	sensor->Open();
	sensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
		&reader);
	return reader; // TODO
}

void cleanupKinect() {
	SAFE_RELEASE(sensor);
	SAFE_RELEASE(reader);
	SAFE_RELEASE(mapper);
}

void getDepthData(IMultiSourceFrame  *frame, GLfloat *dest) {
	IDepthFrame* depthFrame = nullptr;
	IDepthFrameReference* depthFrameRef = nullptr;
	frame->get_DepthFrameReference(&depthFrameRef);
	depthFrameRef->AcquireFrame(&depthFrame);
	SAFE_RELEASE(depthFrameRef);

	if (!depthFrame)
		return;

	unsigned int sz;
	uint16_t *buf;
	depthFrame->AccessUnderlyingBuffer(&sz, &buf);

	mapper->MapDepthFrameToCameraSpace(DWIDTH * DHEIGHT, buf, DWIDTH * DHEIGHT, depth2xyz);
	mapper->MapDepthFrameToColorSpace(DWIDTH * DHEIGHT, buf, DWIDTH * DHEIGHT, depth2rgb);

	for (unsigned int i = 0; i < sz; i++) {
		*dest++ = depth2xyz[i].X;
		*dest++ = depth2xyz[i].Y;
		*dest++ = depth2xyz[i].Z;
	}
	
	SAFE_RELEASE(depthFrame);
}

void getColorData(IMultiSourceFrame *frame, GLfloat *dest) {
	IColorFrame* colorFrame = nullptr;
	IColorFrameReference* colorFrameRef = nullptr;
	frame->get_ColorFrameReference(&colorFrameRef);
	colorFrameRef->AcquireFrame(&colorFrame);
	SAFE_RELEASE(colorFrameRef);

	if (!colorFrame)
		return;

	colorFrame->CopyConvertedFrameDataToArray(CWIDTH * CHEIGHT * 4, rgbaBuf, ColorImageFormat_Rgba);

	for (int i = 0; i < DWIDTH * DHEIGHT; ++i) {
		ColorSpacePoint p = depth2rgb[i];
		if (p.X < 0 || p.Y < 0 || p.X > CWIDTH || p.Y > CHEIGHT) { // Check if color pixel within frame
			for (int j = 0; j < 3; ++j)
				*dest++ = 0;
		} else {
			int idx = (int)p.X + CWIDTH * (int)p.Y;
			for (int j = 0; j < 3; ++j)
				*dest++ = rgbaBuf[4 * idx + j] / 255.f;
		}
	}

	SAFE_RELEASE(colorFrame);
}

void getKinectData() {
	IMultiSourceFrame* frame = nullptr;
	if (!SUCCEEDED(reader->AcquireLatestFrame(&frame)))
		return;

	GLfloat* ptr;
	glBindBuffer(GL_ARRAY_BUFFER, vboDepth);
	ptr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr)
		getDepthData(frame, ptr);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, vboColor);
	ptr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr)
		getColorData(frame, ptr);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	
	SAFE_RELEASE(frame);
}

void initOpenGL() {
	// Init GLFW
	glfwInit();
	window = glfwCreateWindow(WIDTH, HEIGHT, "Video Anamorphosis", NULL, NULL);
	glfwMakeContextCurrent(window);
	// Init GLEW
	glewInit();

	// OpenGL flags
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);

	// Compile shaders
	prog = compileProgram();

	// Matrices
	viewMat = glm::lookAt(
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, -1.0f),
		glm::vec3(0.0f, 1.0f, 0.0f)
	);
	projMat = glm::perspective(glm::radians(70.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 1000.0f);
	pvmMat = projMat * viewMat;

	// VAOs and VBOs
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vboDepth);
	glBindBuffer(GL_ARRAY_BUFFER, vboDepth);
	glBufferData(GL_ARRAY_BUFFER, DWIDTH * DHEIGHT * 3 * sizeof(GLfloat), 0, GL_DYNAMIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);

	glGenBuffers(1, &vboColor);
	glBindBuffer(GL_ARRAY_BUFFER, vboColor);
	glBufferData(GL_ARRAY_BUFFER, DWIDTH * DHEIGHT * 3 * sizeof(GLfloat), 0, GL_DYNAMIC_DRAW);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void update(float dt) {
	getKinectData();

	int angleDir = 0;
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) angleDir += 1;
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) angleDir -= 1;
	if (angleDir != 0) angle -= angleDir * speed;
	
	viewMat = glm::lookAt(
		glm::vec3(radius * sinf(angle), 0.0f, radius * (1.0f - cosf(angle))),
		glm::vec3(0.0f, 0.0f, radius),
		glm::vec3(0.0f, 1.0f, 0.0f)
	);
	pvmMat = projMat * viewMat;
}

void draw() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(prog);
	glBindVertexArray(vao);

	glUniformMatrix4fv(glGetUniformLocation(prog, "uPVM"), 1, GL_FALSE, glm::value_ptr(pvmMat));

	glPointSize((GLfloat)HEIGHT / (GLfloat)DHEIGHT);
	glDrawArrays(GL_POINTS, 0, DWIDTH * DHEIGHT * 3 * sizeof(GLfloat));
}

int main() {
	// Init OpenGL
	initOpenGL();
	// Init Kinect
	if (!initKinect()) {
		std::cout << "Can't init Kinect" << std::endl;
		return 1;
	}

	// Main loop
	auto lastFrame = std::chrono::steady_clock::now();
	while (!glfwWindowShouldClose(window)) {
		auto currentFrame = std::chrono::steady_clock::now();
		float dt = std::chrono::duration<float>(currentFrame - lastFrame).count();
		lastFrame = currentFrame;

		update(dt);
		draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwTerminate();
	cleanupKinect();

	return 0;
}
