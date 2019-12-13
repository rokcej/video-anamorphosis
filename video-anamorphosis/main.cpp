#define _CRT_SECURE_NO_WARNINGS

// Kinect
#include <Windows.h>
#include <Ole2.h>
#include <Kinect.h>
// Stdlib
#include <iostream>
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <vector>
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

#define WIDTH 1920 // Window res
#define HEIGHT 1080
#define DWIDTH 512 // Depth sensor res
#define DHEIGHT 424
#define CWIDTH 1920 // Color camera res
#define CHEIGHT 1080

#define SAFE_RELEASE(ptr) { if (ptr) { (ptr)->Release(); (ptr) = nullptr; } }

// OpenGL variables
GLFWwindow* window = nullptr;
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

// Anamorphosis
unsigned char *image = nullptr;
int imageWidth, imageHeight;
float projAngle = 0.0f;
float projFovy = 45.0f;
float radius = 1.0f;

// Interaction
float angle = 0.0f; // Rotation angle
float speed = 0.5f; // Rotation speed
bool anamorphosis = false;

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
		depth2xyz[i].Z *= -1.f;
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

glm::vec3 circlePos(float angleDeg) {
	float angleRad = angleDeg * (float)M_PI / 180.0f;
	return glm::vec3(radius * sinf(angleRad), 0.0f, -radius * (1.0f - cosf(angleRad)));
}

void getAnamorphicData(GLfloat* dest) {
	glm::vec3 projPos = circlePos(projAngle);
	glm::vec3 projTarget(0.0f, 0.0f, -radius);
	glm::vec3 projDir = glm::normalize(projTarget - projPos);
	glm::vec3 projUp(0.0f, 1.0f, 0.0f);
	glm::vec3 projRight = glm::normalize(glm::cross(projDir, projUp)); // Should already be normal TODO REMOVE

	float ratio = (float)imageWidth / (float)imageHeight;
	float vMax = tanf(projFovy * (float)M_PI / 360.0f);
	float uMax = vMax * ratio;

	for (int i = 0; i < DWIDTH * DHEIGHT; ++i) {
		CameraSpacePoint point = depth2xyz[i];
		glm::vec3 rayDir = glm::vec3(point.X, point.Y, point.Z) - projPos;

		float t = glm::dot(projDir, projDir) / glm::dot(projDir, rayDir);
		glm::vec3 uv =  t * rayDir - projDir;
		float u = glm::dot(uv, projRight);
		float v = glm::dot(uv, -projUp);

		int x = (int)roundf((u / uMax + 1.0f) * 0.5f * (float)(imageWidth-1));
		int y = (int)roundf((v / vMax + 1.0f) * 0.5f * (float)(imageHeight-1));

		if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight) {
			int idx = (y * imageWidth + x) * 3;
			for (int j = 0; j < 3; ++j)
				*dest++ = (float)image[idx + j] / 255.0f;
		} else {
			for (int j = 0; j < 3; ++j)
				*dest++ *= 0.4f;
				//*dest++ = 0.0f;
		}
	}
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
	if (ptr) {
		//if (!anamorphosis) // Map color to points
			getColorData(frame, ptr);
		if (anamorphosis) // Map anamorphic image to points
			getAnamorphicData(ptr);
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
	
	SAFE_RELEASE(frame);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	switch (key) {
		case GLFW_KEY_ESCAPE: // Exit
			if (action == GLFW_PRESS)
				glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_SPACE: // Toggle
			if (action == GLFW_PRESS)
				anamorphosis = !anamorphosis;
			break;
		case GLFW_KEY_1: // Projector view
			if (action == GLFW_PRESS)
				angle = 0.0f;
			break;
		case GLFW_KEY_2: // Observer view
			if (action == GLFW_PRESS)
				angle = projAngle;
			break;
		case GLFW_KEY_ENTER: // Apply projection to current angle
			if (action == GLFW_PRESS)
				projAngle = angle;
			break;
	}
}

void initOpenGL() {
	// Init GLFW
	glfwInit();
	window = glfwCreateWindow(WIDTH, HEIGHT, "Video Anamorphosis", glfwGetPrimaryMonitor(), NULL);
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, keyCallback);
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
	projMat = glm::perspective(glm::radians(60.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 1000.0f);
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

	// Rotation
	int angleDir = 0;
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) angleDir += 1;
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) angleDir -= 1;
	if (angleDir != 0) angle += angleDir * speed;

	int fovDir = 0;
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) fovDir += 1;
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) fovDir -= 1;
	if (fovDir != 0) projFovy += fovDir * speed;
	
	viewMat = glm::lookAt(
		circlePos(angle),
		glm::vec3(0.0f, 0.0f, -radius),
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

	glPointSize((GLfloat)HEIGHT / (GLfloat)DHEIGHT * 1.2f);
	glDrawArrays(GL_POINTS, 0, DWIDTH * DHEIGHT * 3 * sizeof(GLfloat));
}

void init() {
	int n;
	image = stbi_load("image.jpg", &imageWidth, &imageHeight, &n, 3);
}

void cleanup() {
	stbi_image_free(image);
}

int main() {
	// Init OpenGL
	initOpenGL();
	// Init Kinect
	if (!initKinect()) {
		std::cout << "Can't init Kinect" << std::endl;
		return 1;
	}
	// Init else
	init();

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
	cleanup();

	return 0;
}
