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
#include <thread>
//#include <cstdint>
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

#define WIDTH 512
#define HEIGHT 424

#define SAFE_RELEASE(ptr) { if (ptr) { (ptr)->Release(); (ptr) = nullptr; } }

// GLFW variables
GLFWwindow* window = nullptr;

// OpenGL variables
GLuint prog; // Shader program
GLuint vao, vbo;
GLuint textureId;              // ID of the texture to contain Kinect RGB Data
GLubyte data[WIDTH * HEIGHT * 4];  // BGRA array containing the texture data
std::vector<GLfloat> vertices = {
		 0.5f,  0.5f, -5.0f,  // top right
		-0.5f, -0.5f, -5.0f,  // bottom left
		 0.5f, -0.5f, -5.0f  // bottom right
};
glm::mat4 viewMat, projMat, pvmMat;
glm::vec3 pos, forward, up;



// Kinect variables
IKinectSensor* kinectSensor = nullptr; // Kinect sensor
IDepthFrameReader* depthFrameReader = nullptr; // Kinect depth data reader
uint16_t* depthBuffer = nullptr;
float kinectFovy = 60.0f;
float kinectFovx = 70.6f;

bool initKinect() {
	if (FAILED(GetDefaultKinectSensor(&kinectSensor)))
		return false;

	if (!kinectSensor)
		return false;

	kinectSensor->Open();

	IDepthFrameSource* depthFrameSource = nullptr;
	if (FAILED(kinectSensor->get_DepthFrameSource(&depthFrameSource)))
		return false;

	if (FAILED(depthFrameSource->OpenReader(&depthFrameReader)))
		return false;

	SAFE_RELEASE(depthFrameSource);

	depthBuffer = new uint16_t[512 * 424];

	return true;
}

void cleanupKinect() {
	delete[] depthBuffer;
	SAFE_RELEASE(depthFrameReader);
	SAFE_RELEASE(kinectSensor);
}

void getKinectData() {
	IDepthFrame* depthFrame = nullptr;

	int attempts = 100;
	do {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	} while (FAILED(depthFrameReader->AcquireLatestFrame(&depthFrame)) && --attempts > 0);
	if (attempts == 0) {
		std::cout << "Failed to get depth data from Kinect";
		return;
	}

	if (FAILED(depthFrame->CopyFrameDataToArray(512 * 424, depthBuffer))) {
		SAFE_RELEASE(depthFrame);
		return;
	}

	SAFE_RELEASE(depthFrame);
}

void saveDepthImage() {
	float* pixels = new float[WIDTH * HEIGHT * 3];
	for (int i = 0; i < WIDTH * HEIGHT; ++i) {
		for (int j = 0; j < 3; ++j)
			pixels[3 * i + j] = (depthBuffer[i] % 255) / 255.0f;
	}
	bmpSaveImage(pixels, WIDTH, HEIGHT, "depth.bmp");
	delete[] pixels;
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

	// Matrices and vector
	pos = glm::vec3(0.0f, 0.0f, 0.0f);
	up = glm::vec3(0.0f, 1.0f, 0.0f);
	forward = glm::vec3(0.0f, 0.0f, -1.0f);
	viewMat = glm::lookAt(
		pos,
		pos + forward,
		up
	);
	projMat = glm::perspective(glm::radians(90.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 1000.0f);
	pvmMat = projMat * viewMat;
}

void getMesh() {
	vertices.clear();
	float relWidth = 2.0f * tanf(kinectFovx * M_PI / 360.0f);
	float relHeight = 2.0f * tanf(kinectFovy * M_PI / 360.0f);
	glm::vec3** points = new glm::vec3*[HEIGHT];
	for (int y = 0; y < HEIGHT; ++y) {
		points[y] = new glm::vec3[WIDTH];
		for (int x = 0; x < WIDTH; ++x) {
			int d = depthBuffer[y * WIDTH + x];
			points[y][x] = (float)d * 0.001f * glm::normalize(glm::vec3(
				((float)x / (float)(WIDTH - 1) - 0.5f) * relWidth,
				((float)y / (float)(HEIGHT - 1) - 0.5f) * relHeight,
				-1.0f
			));
		}
	}
	for (int y = 0; y < HEIGHT - 1; ++y) {
		for (int x = 0; x < WIDTH - 1; ++x) {
			int indices[] = { x,y, x,y+1, x+1,y+1, x,y, x+1,y+1, x+1,y };
			for (int i = 0; i < 6; ++i) {
				int ix = indices[2 * i];
				int iy = indices[2 * i + 1];
				for (int j = 0; j < 3; ++j)
					vertices.push_back(points[iy][ix][j]);
			}
		}
	}
	// Free memory
	for (int y = 0; y < HEIGHT - 1; ++y)
		delete[] points[y];
	delete[] points;
}

void loadMesh() {
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	glBindVertexArray(vao);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat), vertices.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void update(float seconds) {
	viewMat = glm::lookAt(
		glm::vec3(5.0f * sinf(seconds), 0.0f, 5.0f * (cosf(seconds) - 1.0f)),
		glm::vec3(0.0f, 0.0f, -5.0f),
		up
	);
	pvmMat = projMat * viewMat;
}

void draw() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(prog);

	glUniformMatrix4fv(glGetUniformLocation(prog, "uPVM"), 1, GL_FALSE, glm::value_ptr(pvmMat));

	glBindVertexArray(vao);

	glDrawArrays(GL_TRIANGLES, 0, 3);
}

int main() {
	// Init OpenGL
	initOpenGL();
	// Init Kinect
	if (!initKinect()) {
		std::cout << "Can't init Kinect" << std::endl;
		return 1;
	}

	getKinectData();
	saveDepthImage();
	getMesh();
	loadMesh();

	// Main loop
	auto startTimer = std::chrono::steady_clock::now();
	while (!glfwWindowShouldClose(window)) {
		float seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - startTimer).count();

		//update(seconds);
		draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwTerminate();
	cleanupKinect();

	return 0;
}
