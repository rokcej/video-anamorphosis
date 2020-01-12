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
#include <algorithm>
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
#include "approx.h"
#include "bmp.h"

#define WIDTH 1920 // Window res
#define HEIGHT 1080
#define DWIDTH 512 // Depth sensor res
#define DHEIGHT 424
#define CWIDTH 1920 // Color camera res
#define CHEIGHT 1080
#define PWIDTH 1920 // Projector res
#define PHEIGHT 1080

#define SAFE_RELEASE(ptr) { if (ptr) { (ptr)->Release(); (ptr) = nullptr; } }

inline int DI(const int x, const int y) { return y * DWIDTH + x; } // Depth index

// Data
float* triangles;
uint8_t* pixels;
uint16_t* depths;

// OpenGL variables
GLFWwindow* window = nullptr;
GLuint prog, progTex; // Shader program
GLuint vao, vboDepth, vboColor;
GLuint vaoTri, vboTriPos, vboTriColor;
GLuint numTri = 0;
GLuint vaoPix, texPix, vboPix, eboTex;
glm::mat4 viewMat, projMat, pvmMat;

// Kinect variables
IKinectSensor *sensor = nullptr; // Kinect sensor
IMultiSourceFrameReader *reader = nullptr; // Kinect depth data reader
ICoordinateMapper *mapper = nullptr;

uint8_t rgbaBuf[CWIDTH * CHEIGHT * 4];
ColorSpacePoint depth2rgb[DWIDTH * DHEIGHT];
CameraSpacePoint depth2xyz[DWIDTH * DHEIGHT];

// Anamorphosis
uint8_t *image = nullptr;
int imageWidth, imageHeight;
float projAngle = 0.0f;
float projFovy = 45.0f;
float radius = 1.0f; // Meters

// Interaction
float angle = 0.0f; // Rotation angle
float speed = 0.5f; // Rotation speed
bool anamorphosis = false;
bool debug = false;
bool wireframe = false;
bool approxMissing = false;

bool saveObject = false;

bool initKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor)
		return false;

	sensor->get_CoordinateMapper(&mapper);
	sensor->Open();
	sensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
		&reader
	);
	return reader; // TODO
}

void cleanupKinect() {
	SAFE_RELEASE(sensor);
	SAFE_RELEASE(reader);
	SAFE_RELEASE(mapper);
}

bool getDepthData(IMultiSourceFrame* frame) {
	IDepthFrame* depthFrame = nullptr;
	IDepthFrameReference* depthFrameRef = nullptr;
	frame->get_DepthFrameReference(&depthFrameRef);
	depthFrameRef->AcquireFrame(&depthFrame);
	SAFE_RELEASE(depthFrameRef);

	if (!depthFrame)
		return false;

	unsigned int sz;
	uint16_t* buf;
	depthFrame->AccessUnderlyingBuffer(&sz, &buf);
	std::copy(buf, buf + sz, depths);

	SAFE_RELEASE(depthFrame);

	return true;
}

bool getColorData(IMultiSourceFrame* frame) {
	IColorFrame* colorFrame = nullptr;
	IColorFrameReference* colorFrameRef = nullptr;
	frame->get_ColorFrameReference(&colorFrameRef);
	colorFrameRef->AcquireFrame(&colorFrame);
	SAFE_RELEASE(colorFrameRef);

	if (!colorFrame)
		return false;

	colorFrame->CopyConvertedFrameDataToArray(CWIDTH * CHEIGHT * 4, rgbaBuf, ColorImageFormat_Rgba);

	SAFE_RELEASE(colorFrame);

	return true;
}

void processDepthData() {
	if (approxMissing)
		approxMissingData(depths, DWIDTH, DHEIGHT);

	uint8_t* pixPtr = pixels;
	for (int y = 0; y < 1080; ++y) {
		for (int x = 0; x < 1920; ++x) {
			int x2 = (x * DWIDTH) / 1920;
			int y2 = (y * DHEIGHT) / 1080;
			int idx = y2 * DWIDTH + x2;
			uint8_t c = depths[idx] % 256;
			*pixPtr++ = c;
			*pixPtr++ = c;
			*pixPtr++ = c;
		}
	}
	glBindTexture(GL_TEXTURE_2D, texPix);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, PWIDTH, PHEIGHT, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	mapper->MapDepthFrameToCameraSpace(DWIDTH * DHEIGHT, depths, DWIDTH * DHEIGHT, depth2xyz);
	mapper->MapDepthFrameToColorSpace(DWIDTH * DHEIGHT, depths, DWIDTH * DHEIGHT, depth2rgb);

	/*float xMax = 0; float xMin = 1000;
	float yMax = 0; float yMin = 1000;
	for (int i = 0; i < sz; ++i) {
		if (buf[i] != 0) {
			glm::vec3 pt = glm::vec3(depth2xyz[i].X, depth2xyz[i].Y, depth2xyz[i].Z);
			pt /= pt.z;
			float rx = ((float)(i % DWIDTH) / (DWIDTH - 1.0f) - 0.5f) * 2.0f;
			float ry = (0.5f - (float)(i / DWIDTH) / (DHEIGHT - 1.0f)) * 2.0f;

			float x = pt.x / rx;
			float y = pt.y / ry;

			float xPhi = acosf(1 / sqrtf(x * x + 1));
			float yPhi = acosf(1 / sqrtf(y * y + 1));
			
			if (xPhi < xMin) xMin = xPhi;
			if (xPhi > xMax) xMax = xPhi;
			if (yPhi < yMin) yMin = yPhi;
			if (yPhi > yMax) yMax = yPhi;

			//std::cout << pt.x << " " << rx << ", " << pt.y << " " << ry << std::endl;
			//std::cout << xPhi << " " << yPhi << std::endl;
		}
	}
	std::cout << xMin << " " << xMax << ", " << yMin << " " << yMax << std::endl;*/

	glBindBuffer(GL_ARRAY_BUFFER, vboDepth);
	GLfloat* vboDepthPtr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (vboDepthPtr) {
		for (unsigned int i = 0; i < DWIDTH * DHEIGHT; i++) {
			depth2xyz[i].Z *= -1.f;
			*vboDepthPtr++ = depth2xyz[i].X;
			*vboDepthPtr++ = depth2xyz[i].Y;
			*vboDepthPtr++ = depth2xyz[i].Z;
		}
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glBindBuffer(GL_ARRAY_BUFFER, vboTriPos);
	GLfloat* vboTriPosPtr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (vboDepthPtr) {
		numTri = 0;
		float* ptr = vboTriPosPtr;
		for (int y = 0; y < DHEIGHT - 1; ++y) {
			for (int x = 0; x < DWIDTH - 1; ++x) {
				int i0 = y * DWIDTH + x;
				int i1 = i0 + DWIDTH;
				int i2 = i1 + 1;
				int i3 = i0 + 1;

				if (depth2xyz[i1].X > -100.0f || depth2xyz[i3].X > -100.0f) {
					if (depth2xyz[i0].X > -100.0f) {
						*ptr++ = depth2xyz[i0].X; *ptr++ = depth2xyz[i0].Y; *ptr++ = depth2xyz[i0].Z;
						*ptr++ = depth2xyz[i1].X; *ptr++ = depth2xyz[i1].Y; *ptr++ = depth2xyz[i1].Z;
						*ptr++ = depth2xyz[i3].X; *ptr++ = depth2xyz[i3].Y; *ptr++ = depth2xyz[i3].Z;
						++numTri;
					}
					if (depth2xyz[i2].X > -100.0f) {
						*ptr++ = depth2xyz[i1].X; *ptr++ = depth2xyz[i1].Y; *ptr++ = depth2xyz[i1].Z;
						*ptr++ = depth2xyz[i2].X; *ptr++ = depth2xyz[i2].Y; *ptr++ = depth2xyz[i2].Z;
						*ptr++ = depth2xyz[i3].X; *ptr++ = depth2xyz[i3].Y; *ptr++ = depth2xyz[i3].Z;
						++numTri;
					}
				}
			}
		}
		if (saveObject) {
			std::ofstream file;
			file.open("object.obj");
			ptr = vboTriPosPtr;
			for (int i = 0; i < numTri * 3; ++i) {
				file << "v " << *ptr++;
				file << " " << *ptr++;
				file << " " << *ptr++ << std::endl;
			}
			int ctr = 1;
			for (int i = 0; i < numTri; ++i) {
				file << "f " << ctr++;
				file << " " << ctr++;
				file << " " << ctr++ << std::endl;
			}
			file.close();
			saveObject = false;
		}
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
}

void processColorData(GLfloat* dest) {
	for (int i = 0; i < DWIDTH * DHEIGHT; ++i) {
		ColorSpacePoint p = depth2rgb[i];
		if (p.X < 0 || p.Y < 0 || p.X > CWIDTH || p.Y > CHEIGHT) { // Check if color pixel within frame
			for (int j = 0; j < 3; ++j)
				*dest++ = 0.0f;
		} else {
			int idx = (int)p.X + CWIDTH * (int)p.Y;
			for (int j = 0; j < 3; ++j)
				*dest++ = rgbaBuf[4 * idx + j] / 255.f;
		}
	}
}

glm::vec3 circlePos(float angleDeg) {
	float angleRad = angleDeg * (float)M_PI / 180.0f;
	return glm::vec3(radius * sinf(angleRad), 0.0f, -radius * (1.0f - cosf(angleRad)));
}

void processAnamorphicData(GLfloat* dest) {
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

	bool depthUpdate = getDepthData(frame);
	bool colorUpdate = getColorData(frame);

	if (depthUpdate)
		processDepthData();

	glBindBuffer(GL_ARRAY_BUFFER, vboColor);
	GLfloat* ptr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr) {
		//if (!anamorphosis) // Map color to points
		if (colorUpdate)
			processColorData(ptr);
		if (anamorphosis) // Map anamorphic image to points
			processAnamorphicData(ptr);
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
		case GLFW_KEY_D: // Toggle debug mode
			if (action == GLFW_PRESS)
				debug = !debug;
			break;
		case GLFW_KEY_W: // Toggle wireframe mode
			if (action == GLFW_PRESS)
				wireframe = !wireframe;
			break;
		case GLFW_KEY_M: // Toggle getting missing depth data
			if (action == GLFW_PRESS)
				approxMissing = !approxMissing;
			break;
	}
}

void initOpenGL() {
	// Init GLFW
	glfwInit();
	window = glfwCreateWindow(WIDTH, HEIGHT, "Video Anamorphosis", NULL, NULL); // glfwGetPrimaryMonitor()
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, keyCallback);
	// Init GLEW
	glewInit();

	// OpenGL flags
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);

	// Compile shaders
	prog = compileProgram(vsSource, fsSource);
	progTex = compileProgram(vsSourceTex, fsSourceTex);

	// Matrices
	viewMat = glm::lookAt(
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, -1.0f),
		glm::vec3(0.0f, 1.0f, 0.0f)
	);
	projMat = glm::perspective(glm::radians(60.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 1000.0f);
	pvmMat = projMat * viewMat;

	// VAOs and VBOs
	// Points
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

	// Triangles
	glGenVertexArrays(1, &vaoTri);
	glBindVertexArray(vaoTri);

	glGenBuffers(1, &vboTriPos);
	glBindBuffer(GL_ARRAY_BUFFER, vboTriPos);
	glBufferData(GL_ARRAY_BUFFER, (DWIDTH-1) * (DHEIGHT-1) * 2 * 3 * 3 * sizeof(GLfloat), 0, GL_DYNAMIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);

	glGenBuffers(1, &vboTriColor);
	glBindBuffer(GL_ARRAY_BUFFER, vboTriColor);
	glBufferData(GL_ARRAY_BUFFER, (DWIDTH-1) * (DHEIGHT-1) * 2 * 3 * 3 * sizeof(GLfloat), 0, GL_DYNAMIC_DRAW);
	GLfloat* vboTriColorPtr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	for (int i = 0; i < (DWIDTH - 1) * (DHEIGHT - 1) * 2 * 3 * 3; ++i)
		*vboTriColorPtr++ = 1.0f;
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);

	// Pixels
	glGenVertexArrays(1, &vaoPix);
	glBindVertexArray(vaoPix);

	glGenTextures(1, &texPix);
	glBindTexture(GL_TEXTURE_2D, texPix);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, PWIDTH, PHEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	glActiveTexture(GL_TEXTURE0);

	GLfloat pixVertices[] = {
		// Pos				// Tex coord (flipped vertically)
		-1.0f, +1.0f, 0.0f,	0.0f, 0.0f, // Top left
		-1.0f, -1.0f, 0.0f,	0.0f, 1.0f, // Bottom left
		+1.0f, -1.0f, 0.0f,	1.0f, 1.0f, // Bottom right
		+1.0f, +1.0f, 0.0f,	1.0f, 0.0f, // Top right
	};
	GLuint pixIndices[] = {
		0, 1, 2,
		0, 2, 3
	};

	glGenBuffers(1, &vboPix);
	glBindBuffer(GL_ARRAY_BUFFER, vboPix);
	glBufferData(GL_ARRAY_BUFFER, sizeof(pixVertices), pixVertices, GL_STATIC_DRAW);

	glGenBuffers(1, &eboTex);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboTex);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(pixIndices), pixIndices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));

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

	if (debug) {
		glUseProgram(prog);

		if (wireframe) {
			// Mesh
			glBindVertexArray(vaoTri);
			glUniformMatrix4fv(glGetUniformLocation(prog, "uPVM"), 1, GL_FALSE, glm::value_ptr(pvmMat));
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			//glDrawArrays(GL_TRIANGLES, 0, (DWIDTH - 1) * (DHEIGHT - 1) * 2 * 3 * 3 * sizeof(GLfloat));
			glDrawArrays(GL_TRIANGLES, 0, numTri * 3 * sizeof(GLfloat));
		}
		else {
			// Points
			glBindVertexArray(vao);
			glUniformMatrix4fv(glGetUniformLocation(prog, "uPVM"), 1, GL_FALSE, glm::value_ptr(pvmMat));
			glPointSize((GLfloat)HEIGHT / (GLfloat)DHEIGHT * 1.2f);
			glDrawArrays(GL_POINTS, 0, DWIDTH * DHEIGHT * 3 * sizeof(GLfloat));
		}
	} else {
		glUseProgram(progTex);

		// Pixels
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBindVertexArray(vaoPix);
		glUniform1i(glGetUniformLocation(progTex, "uTex"), 0);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	}
}

void init() {
	int n;
	image = stbi_load("image.jpg", &imageWidth, &imageHeight, &n, 3);
	triangles = new float[(DWIDTH - 1) * (DHEIGHT - 1) * 2 * 3 * 3];
	pixels = new uint8_t[PWIDTH * PHEIGHT * 3];
	depths = new uint16_t[DWIDTH * DHEIGHT];
	approxes = new Approx[DWIDTH * DHEIGHT];
}

void cleanup() {
	stbi_image_free(image);
	delete[] triangles;
	delete[] pixels;
	delete[] depths;
	delete[] approxes;
}

int main() {
	// My init
	init();
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
	cleanup();

	return 0;
}
