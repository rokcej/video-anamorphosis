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
#include "approx.h"
#include "video.h"
#include "bmp.h"

#define IMAGE_PATH "image.jpg"
#define VIDEO_PATH "video2.mp4"

#define WIDTH 1600 // Window res
#define HEIGHT 1200
#define DWIDTH 512 // Depth sensor res
#define DHEIGHT 424
#define CWIDTH 1920 // Color camera res
#define CHEIGHT 1080
#define PWIDTH 1600 // Projector res
#define PHEIGHT 1200

#define NUM_THREADS 8

constexpr float projFovX = 36.86989761f; // 37.55606644f;
constexpr float projFovY = 28.07248694f; // 28.61110369f;

#define SAFE_RELEASE(ptr) { if (ptr) { (ptr)->Release(); (ptr) = nullptr; } }

// Data
uint8_t* pixels;
uint16_t* depths;
int* pixelMap;

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
float depthFovX = 0, depthFovY = 0;

// Anamorphosis
uint8_t *imageData = nullptr;
int imageWidth, imageHeight;
float anamorphicAngle = 60.0f;
float anamorphicFovY = 10.0f;
float radius = 1.2f; // Meters

// Interaction
float rotAngle = 0.0f; // Rotation angle
float rotSpeed = 30.0f; // Rotation speed
float zoomStep = 1.02f;
bool anamorphosis = false;
bool debug = false;
bool wireframe = false;
bool approxMissing = false;
int calibrate = 0;
bool record = true;
bool kinectMap = true;
bool video = true;

bool saveObject = false;

inline int DI(const int x, const int y) { return y * DWIDTH + x; } // Depth index
inline float RAD(const float deg) { return deg * ((float)M_PI / 180.0f); } // Degrees to radians
inline float DEG(const float rad) { return rad * (180.0f / (float)M_PI); } // Radians to degrees
inline int SAMPLE_WIDTH() { return video ? videoWidth : imageWidth; }
inline int SAMPLE_HEIGHT() { return video ? videoHeight : imageHeight; }
inline uint8_t SAMPLE_DATA(int x, int y, int off) { return video ? videoData[4 * (y * videoWidth + x) + off] : imageData[3 * (y * imageWidth + x) + off]; }
inline glm::vec3 SAMPLE_AVG(float sx, float sy) {
	if (sx < 0.0f || sx > SAMPLE_WIDTH() - 1.0f || sy < 0 || sy > SAMPLE_HEIGHT() - 1.0f)
		return glm::vec3(0.0f, 0.0f, 0.0f);

	int xUp = (int)ceilf(sx);
	int xDown = (int)sx;
	int yUp = (int)ceilf(sy);
	int yDown = (int)sy;

	glm::vec3 p00(SAMPLE_DATA(xDown, yDown, 0), SAMPLE_DATA(xDown, yDown, 1), SAMPLE_DATA(xDown, yDown, 2));
	glm::vec3 p01(SAMPLE_DATA(xUp, yDown, 0), SAMPLE_DATA(xUp, yDown, 1), SAMPLE_DATA(xUp, yDown, 2));
	glm::vec3 p10(SAMPLE_DATA(xDown, yUp, 0), SAMPLE_DATA(xDown, yUp, 1), SAMPLE_DATA(xDown, yUp, 2));
	glm::vec3 p11(SAMPLE_DATA(xUp, yUp, 0), SAMPLE_DATA(xUp, yUp, 1), SAMPLE_DATA(xUp, yUp, 2));

	float wx = sx - (float)xDown;
	float wy = sy - (float)yDown;

	return (p00 * (2.0f - wx - wy) + p01 * (1.0f + wx - wy) + p10 * (1.0f - wx + wy) + p11 * (wx + wy)) * 0.25f;
}

glm::vec3 circlePos(float angleDeg) {
	float angleRad = RAD(angleDeg);
	return glm::vec3(0.0f, radius * sinf(angleRad), -radius * (1.0f - cosf(angleRad)));
	//return glm::vec3(radius * sinf(angleRad), 0.0f, -radius * (1.0f - cosf(angleRad)));
}

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

	depthFrame->CopyFrameDataToArray(DWIDTH * DHEIGHT, depths);

	IFrameDescription* depthDescription = nullptr;
	depthFrame->get_FrameDescription(&depthDescription);
	if (depthDescription) {
		depthDescription->get_HorizontalFieldOfView(&depthFovX);
		depthDescription->get_VerticalFieldOfView(&depthFovY);
	}

	SAFE_RELEASE(depthDescription);
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

void myDepthTo3D() {
	float uLen = tanf(RAD(depthFovX * 0.5f));
	float vLen = tanf(RAD(depthFovY * 0.5f));

	for (int y = 0; y < DHEIGHT; ++y) {
		for (int x = 0; x < DWIDTH; ++x) {
			int idx = y * DWIDTH + x;
			float d = (float)depths[idx] / 1000.0f; // mm => m

			float u = (float)x / (DWIDTH - 1.0f) * 2.0f - 1.0f;
			float v = 1.0f - (float)y / (DHEIGHT - 1.0f) * 2.0f;

			glm::vec3 dir(u * uLen, v * vLen, 1.0f);
			glm::vec3 point = dir * d;

			depth2xyz[idx].X = point.x;
			depth2xyz[idx].Y = point.y;
			depth2xyz[idx].Z = point.z;
		}
	}
}

void processDepthData() {
	if (approxMissing)
		approxMissingData(depths, DWIDTH, DHEIGHT);

	if (kinectMap)
		mapper->MapDepthFrameToCameraSpace(DWIDTH * DHEIGHT, depths, DWIDTH * DHEIGHT, depth2xyz);
	else
		myDepthTo3D();
	mapper->MapDepthFrameToColorSpace(DWIDTH * DHEIGHT, depths, DWIDTH * DHEIGHT, depth2rgb);

	glBindBuffer(GL_ARRAY_BUFFER, vboDepth);
	GLfloat* vboDepthPtr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (vboDepthPtr) {
		for (unsigned int i = 0; i < DWIDTH * DHEIGHT; i++) {
			depth2xyz[i].Z *= -1.f;
			//depth2xyz[i].X *= -1.f;
			*vboDepthPtr++ = depth2xyz[i].X;
			*vboDepthPtr++ = depth2xyz[i].Y;
			*vboDepthPtr++ = depth2xyz[i].Z;
		}
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);

	// Create mesh from points
	if (wireframe) {
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
				for (unsigned int i = 0; i < numTri * 3; ++i) {
					file << "v " << *ptr++;
					file << " " << *ptr++;
					file << " " << *ptr++ << std::endl;
				}
				int ctr = 1;
				for (unsigned int i = 0; i < numTri; ++i) {
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
}

void getPixelsThread(int offset, glm::vec3 obsPos, glm::vec3 obsDir, glm::vec3 obsRight, glm::vec3 obsUp, float uMax, float vMax) {
	for (int py = offset; py < PHEIGHT; py += NUM_THREADS) {
		for (int px = 0; px < PWIDTH; ++px) {
			int pIdx = 3 * ((py + 1) * PWIDTH - 1 - px);

			for (int i = 0; i < 3; ++i)
				pixels[pIdx + i] = 0;

			float x = 2.0f * (float)px / (PWIDTH - 1.0f) - 1.0f;
			float y = 1.0f - 2.0f * (float)py / (PHEIGHT - 1.0f);

			x = x * tanf(RAD(projFovX * 0.5f)) / tanf(RAD(depthFovX * 0.5f));
			y = y * tanf(RAD(projFovY * 0.5f)) / tanf(RAD(depthFovY * 0.5f));

			x = (x + 1.0f) * 0.5f * (DWIDTH - 1.0f);
			y = (1.0f - y) * 0.5f * (DHEIGHT - 1.0f);

			if (x >= 0.0f && x < DWIDTH - 1.0f && y >= 0.0f && y <= DHEIGHT - 1.0f) {
				int xUp = (int)ceilf(x);
				int xDown = (int)x;
				int yUp = (int)ceilf(y);
				int yDown = (int)y;

				CameraSpacePoint csp00 = depth2xyz[yDown * DWIDTH + xDown];
				CameraSpacePoint csp01 = depth2xyz[yDown * DWIDTH + xUp];
				CameraSpacePoint csp10 = depth2xyz[yUp * DWIDTH + xDown];
				CameraSpacePoint csp11 = depth2xyz[yUp * DWIDTH + xUp];

				glm::vec3 p00(csp00.X, csp00.Y, csp00.Z);
				glm::vec3 p01(csp01.X, csp01.Y, csp01.Z);
				glm::vec3 p10(csp10.X, csp10.Y, csp10.Z);
				glm::vec3 p11(csp11.X, csp11.Y, csp11.Z);

				float wx = x - (float)xDown;
				float wy = y - (float)yDown;

				glm::vec3 point = (p00 * (2.0f - wx - wy) + p01 * (1.0f + wx - wy) + p10 * (1.0f - wx + wy) + p11 * (wx + wy)) * 0.25f;

				glm::vec3 rayDir = point - obsPos;

				float t = glm::dot(obsDir, obsDir) / glm::dot(obsDir, rayDir);
				glm::vec3 uv = t * rayDir - obsDir;
				float u = glm::dot(uv, obsRight);
				float v = glm::dot(uv, -obsUp);

				int sx = (int)roundf((u / uMax + 1.0f) * 0.5f * (float)(SAMPLE_WIDTH() - 1));
				int sy = (int)roundf((v / vMax + 1.0f) * 0.5f * (float)(SAMPLE_HEIGHT() - 1));

				pixelMap[2 * (py * PWIDTH + px)] = sx;
				pixelMap[2 * (py * PWIDTH + px) + 1] = sy;

				if (sx >= 0 && sx <= SAMPLE_WIDTH() - 1 && sy >= 0 && sy <= SAMPLE_HEIGHT() - 1) {
					for (int j = 0; j < 3; ++j)
						pixels[pIdx + j] = SAMPLE_DATA(sx, sy, j);
				}
				
			} else {
				pixelMap[2 * (py * PWIDTH + px)] = -1;
				pixelMap[2 * (py * PWIDTH + px) + 1] = -1;
			}
		}
	}
}

void getPixels() {
	glm::vec3 obsPos = circlePos(anamorphicAngle);
	glm::vec3 obsTarget(0.0f, 0.0f, -radius);
	glm::vec3 obsDir = glm::normalize(obsTarget - obsPos);
	//glm::vec3 projUp(0.0f, 1.0f, 0.0f);
	//glm::vec3 projRight = glm::normalize(glm::cross(projDir, projUp)); // Should already be normal TODO REMOVE
	glm::vec3 obsRight(1.0f, 0.0f, 0.0f);
	glm::vec3 obsUp = glm::normalize(glm::cross(obsRight, obsDir));

	float ratio = (float)SAMPLE_WIDTH() / (float)SAMPLE_HEIGHT();
	float vMax = tanf(RAD(anamorphicFovY * 0.5f));
	float uMax = vMax * ratio;

	std::thread threads[NUM_THREADS];
	for (int i = 0; i < NUM_THREADS; ++i)
		threads[i] = std::thread(getPixelsThread, i, obsPos, obsDir, obsRight, obsUp, uMax, vMax);
	for (int i = 0; i < NUM_THREADS; ++i)
		threads[i].join();

	/*for (int py = 0; py < PHEIGHT; ++py) {
		for (int px = PWIDTH - 1; px >= 0; --px) {
			int pIdx = 3 * (py * PWIDTH + px);
			for (int i = 0; i < 3; ++i)
				pixels[pIdx + i] = 0;

			float x = 2.0f * (float)px / (PWIDTH - 1.0f) - 1.0f;
			float y = 1.0f - 2.0f * (float)py / (PHEIGHT - 1.0f);

			x = x * tanf(RAD(projFovX * 0.5f)) / tanf(RAD(depthFovX * 0.5f));
			y = y * tanf(RAD(projFovY * 0.5f)) / tanf(RAD(depthFovY * 0.5f));

			x = (x + 1.0f) * 0.5f * (DWIDTH - 1.0f);
			y = (1.0f - y) * 0.5f * (DHEIGHT - 1.0f);

			if (x >= 0.0f && x < DWIDTH - 1.0f && y >= 0.0f && y <= DHEIGHT - 1.0f) {
				int dx = (int)x;
				int dy = (int)y;

				int xUp = (int)ceilf(x);
				int xDown = (int)x;
				int yUp = (int)ceilf(y);
				int yDown = (int)y;

				CameraSpacePoint csp00 = depth2xyz[yDown * DWIDTH + xDown];
				CameraSpacePoint csp01 = depth2xyz[yDown * DWIDTH + xUp];
				CameraSpacePoint csp10 = depth2xyz[yUp * DWIDTH + xDown];
				CameraSpacePoint csp11 = depth2xyz[yUp * DWIDTH + xUp];

				glm::vec3 p00(csp00.X, csp00.Y, csp00.Z);
				glm::vec3 p01(csp01.X, csp01.Y, csp01.Z);
				glm::vec3 p10(csp10.X, csp10.Y, csp10.Z);
				glm::vec3 p11(csp11.X, csp11.Y, csp11.Z);

				float wx = x - (float)xDown;
				float wy = y - (float)yDown;

				glm::vec3 point = (p00 * (2.0f - wx - wy) + p01 * (1.0f + wx - wy) + p10 * (1.0f - wx + wy) + p11 * (wx + wy)) * 0.25f;
				
				//point.Y += 0.08f; // TODO: Shift to projector

				glm::vec3 rayDir = point - obsPos;

				float t = glm::dot(obsDir, obsDir) / glm::dot(obsDir, rayDir);
				glm::vec3 uv = t * rayDir - obsDir;
				float u = glm::dot(uv, obsRight);
				float v = glm::dot(uv, -obsUp);

				int x = (int)roundf((u / uMax + 1.0f) * 0.5f * (float)(imageWidth - 1));
				int y = (int)roundf((v / vMax + 1.0f) * 0.5f * (float)(imageHeight - 1));

				if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight) {
					int idx = (y * imageWidth + x) * 3;
					for (int j = 0; j < 3; ++j)
						pixels[pIdx + j] = image[idx + j];
						//*ptr++ = image[idx + j];
				} else {
					for (int j = 0; j < 3; ++j)
						pixels[pIdx + j] = 0;
						//*ptr++ *= 0;
				}
			}
		}
	}*/
}

void mapPixelsThread(int offset) {
	for (int y = offset; y < PHEIGHT; y += NUM_THREADS) {
		for (int x = 0; x < PWIDTH; ++x) {
			int sx = (int)pixelMap[2 * (y * PWIDTH + x)];
			int sy = (int)pixelMap[2 * (y * PWIDTH + x) + 1];
			if (!(sx >= 0 && sx <= SAMPLE_WIDTH() - 1 && sy >= 0 && sy <= SAMPLE_HEIGHT() - 1))
				continue;

			int idx = sy * SAMPLE_WIDTH() + sx;
			for (int j = 0; j < 3; ++j)
				pixels[3 * ((y + 1) * PWIDTH - 1 - x) + j] = SAMPLE_DATA(sx, sy, j);
		}
	}
}

void mapPixels() {
	std::thread threads[NUM_THREADS];
	for (int i = 0; i < NUM_THREADS; ++i)
		threads[i] = std::thread(mapPixelsThread, i);
	for (int i = 0; i < NUM_THREADS; ++i)
		threads[i].join();
}

void processColorData(GLfloat* dest) {
	GLfloat* ptr = dest;
	for (int i = 0; i < DWIDTH * DHEIGHT; ++i) {
		ColorSpacePoint p = depth2rgb[i];
		if (p.X < 0 || p.Y < 0 || p.X > CWIDTH || p.Y > CHEIGHT) { // Check if color pixel within frame
			for (int j = 0; j < 3; ++j)
				*ptr++ = 0.0f;
		} else {
			int idx = (int)p.X + CWIDTH * (int)p.Y;
			for (int j = 0; j < 3; ++j)
				*ptr++ = rgbaBuf[4 * idx + j] / 255.f;
		}
	}

	if (calibrate) {
		int xIncrement = calibrate == 1 ? 1 : PWIDTH - 1;
		int yIncrement = calibrate == 1 ? 1 : PHEIGHT - 1;
		for (int py = 0; py < PHEIGHT; py += yIncrement) {
			for (int px = PWIDTH - 1; px >= 0; px -= xIncrement) {
				float x = 2.0f * (float)px / (PWIDTH - 1.0f) - 1.0f;
				float y = 1.0f - 2.0f * (float)py / (PHEIGHT - 1.0f);

				x = x * tanf(RAD(projFovX * 0.5f)) / tanf(RAD(depthFovX * 0.5f));
				y = y * tanf(RAD(projFovY * 0.5f)) / tanf(RAD(depthFovY * 0.5f));

				x = (x + 1.0f) * 0.5f * (DWIDTH - 1.0f);
				y = (1.0f - y) * 0.5f * (DHEIGHT - 1.0f);

				int dx = (int)x;
				int dy = (int)y;

				if (dx >= 0 && dx < DWIDTH && dy >= 0 && dy < DHEIGHT) {
					int idx = dy * DWIDTH + dx;
					dest[3 * idx] = 1.0f;
					dest[3 * idx + 1] = 0.0f;
					dest[3 * idx + 2] = 0.0f;
					continue;
				}
			}
		}
	}
}

void getAnamorphicPointsThread(GLfloat* dest, int offset, glm::vec3 obsPos, glm::vec3 obsDir, glm::vec3 obsRight, glm::vec3 obsUp, float uMax, float vMax) {
	dest += offset * 3;
	for (int i = offset; i < DWIDTH * DHEIGHT; i += NUM_THREADS) {
		CameraSpacePoint point = depth2xyz[i];
		glm::vec3 rayDir = glm::vec3(point.X, point.Y, point.Z) - obsPos;

		float t = glm::dot(obsDir, obsDir) / glm::dot(obsDir, rayDir);
		glm::vec3 uv = t * rayDir - obsDir;
		float u = glm::dot(uv, obsRight);
		float v = glm::dot(uv, -obsUp);

		int x = (int)roundf((u / uMax + 1.0f) * 0.5f * (float)(SAMPLE_WIDTH() - 1));
		int y = (int)roundf((v / vMax + 1.0f) * 0.5f * (float)(SAMPLE_HEIGHT() - 1));

		if (x >= 0 && x < SAMPLE_WIDTH() && y >= 0 && y < SAMPLE_HEIGHT()) {
			for (int j = 0; j < 3; ++j)
				dest[j] = (float)SAMPLE_DATA(x, y, j) / 255.0f;
		} else {
			for (int j = 0; j < 3; ++j)
				dest[j] *= 0.4f;
		}
		dest += NUM_THREADS * 3;
	}
}

void getAnamorphicPoints(GLfloat* dest) {
	glm::vec3 obsPos = circlePos(anamorphicAngle);
	glm::vec3 obsTarget(0.0f, 0.0f, -radius);
	glm::vec3 obsDir = glm::normalize(obsTarget - obsPos);
	//glm::vec3 projUp(0.0f, 1.0f, 0.0f);
	//glm::vec3 projRight = glm::normalize(glm::cross(projDir, projUp)); // Should already be normal TODO REMOVE
	glm::vec3 obsRight(1.0f, 0.0f, 0.0f);
	glm::vec3 obsUp = glm::normalize(glm::cross(obsRight, obsDir));

	float ratio = (float)SAMPLE_WIDTH() / (float)SAMPLE_HEIGHT();
	float vMax = tanf(RAD(anamorphicFovY * 0.5f));
	float uMax = vMax * ratio;

	std::thread threads[NUM_THREADS];
	for (int i = 0; i < NUM_THREADS; ++i)
		threads[i] = std::thread(getAnamorphicPointsThread, dest, i, obsPos, obsDir, obsRight, obsUp, uMax, vMax);
	for (int i = 0; i < NUM_THREADS; ++i)
		threads[i].join();

	/*for (int i = 0; i < DWIDTH * DHEIGHT; ++i) {
		CameraSpacePoint point = depth2xyz[i];
		glm::vec3 rayDir = glm::vec3(point.X, point.Y, point.Z) - obsPos;

		float t = glm::dot(obsDir, obsDir) / glm::dot(obsDir, rayDir);
		glm::vec3 uv =  t * rayDir - obsDir;
		float u = glm::dot(uv, obsRight);
		float v = glm::dot(uv, -obsUp);

		int x = (int)roundf((u / uMax + 1.0f) * 0.5f * (float)(SAMPLE_WIDTH()-1));
		int y = (int)roundf((v / vMax + 1.0f) * 0.5f * (float)(SAMPLE_HEIGHT()-1));

		if (x >= 0 && x < SAMPLE_WIDTH() && y >= 0 && y < SAMPLE_HEIGHT()) {
			for (int j = 0; j < 3; ++j)
				*dest++ = (float)SAMPLE_DATA(x, y, j) / 255.0f;
		} else {
			for (int j = 0; j < 3; ++j)
				*dest++ *= 0.4f;
				//*dest++ = 0.0f;
		}
	}*/
}

void getKinectData() {
	IMultiSourceFrame* frame = nullptr;
	if (!SUCCEEDED(reader->AcquireLatestFrame(&frame)))
		return;

	bool depthUpdate = getDepthData(frame);
	bool colorUpdate = getColorData(frame);

	if (depthUpdate) {
		processDepthData();
		if (!debug) {
			getPixels();
			glBindTexture(GL_TEXTURE_2D, texPix);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, PWIDTH, PHEIGHT, GL_RGB, GL_UNSIGNED_BYTE, pixels);
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, vboColor);
	GLfloat* ptr = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	if (ptr) {
		//if (!anamorphosis) // Map color to points
		if (colorUpdate && debug)
			processColorData(ptr);
		if (anamorphosis) // Map anamorphic image to points
			getAnamorphicPoints(ptr);
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
				rotAngle = 0.0f;
			break;
		case GLFW_KEY_2: // Observer view
			if (action == GLFW_PRESS)
				rotAngle = anamorphicAngle;
			break;
		case GLFW_KEY_ENTER: // Apply projection to current angle
			if (action == GLFW_PRESS)
				anamorphicAngle = rotAngle;
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
		case GLFW_KEY_C: // Toggle calibration mode
			if (action == GLFW_PRESS)
				calibrate = (calibrate + 1) % 3;
			break;
		case GLFW_KEY_R: // Toggle recording mode
			if (action == GLFW_PRESS)
				record = !record;
			break;
		case GLFW_KEY_K: // Toggle mapping mode
			if (action == GLFW_PRESS)
				kinectMap = !kinectMap;
			break;
		case GLFW_KEY_V: // Toggle video mode
			if (action == GLFW_PRESS)
				video = !video;
			break;
	}
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
	if (yoffset > 0.0) {
		radius *= powf(zoomStep, (float)yoffset);
	} else if (yoffset < 0.0) {
		radius /= powf(zoomStep, -(float)yoffset);
	}
}

void initOpenGL() {
	// Init GLFW
	glfwInit();

	// Create window
	int monitorCount;
	GLFWmonitor** monitors = glfwGetMonitors(&monitorCount);
	GLFWmonitor* monitor = monitors[monitorCount - 1];
	//monitor = glfwGetPrimaryMonitor();
	
	window = glfwCreateWindow(WIDTH, HEIGHT, "Video Anamorphosis", monitor, NULL); // glfwGetPrimaryMonitor()
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, keyCallback);
	glfwSetScrollCallback(window, scrollCallback);
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

void update(float dt) {// Video
	int64_t pts;
	if (video) {
		if (!record) {
			mapPixels();
		}
		if (!getVideoFrame(&videoState, videoData, &pts)) {
			printf("Couldn't load video frame\n");
		} else {
			glBindTexture(GL_TEXTURE_2D, texPix);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, PWIDTH, PHEIGHT, GL_RGB, GL_UNSIGNED_BYTE, pixels);
		}
	}

	// Kinect
	if (record)
		getKinectData();

	// Rotation
	int angleDir = 0;
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) angleDir += 1;
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) angleDir -= 1;
	if (angleDir != 0) rotAngle += angleDir * rotSpeed * dt;

	int fovDir = 0;
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) fovDir += 1;
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) fovDir -= 1;
	if (fovDir != 0) anamorphicFovY += fovDir * dt;
	
	glm::vec3 eye = circlePos(rotAngle);
	glm::vec3 center(0.0f, 0.0f, -radius);
	glm::vec3 up = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), center - eye);
	viewMat = glm::lookAt(eye, center, up);
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
		} else {
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
	imageData = stbi_load(IMAGE_PATH, &imageWidth, &imageHeight, &n, 3);
	pixels = new uint8_t[PWIDTH * PHEIGHT * 3];
	depths = new uint16_t[DWIDTH * DHEIGHT];
	approxes = new Approx[DWIDTH * DHEIGHT];
	pixelMap = new int[PWIDTH * PHEIGHT * 2];
}

void cleanup() {
	stbi_image_free(imageData);
	delete[] pixels;
	delete[] depths;
	delete[] approxes;
	delete[] pixelMap;
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
	// Init video
	if (!initVideo(&videoState, VIDEO_PATH)) {
		printf("Couldn't open video file\n");
		return 1;
	}

	// Main loop
	auto lastFrame = std::chrono::steady_clock::now();
	auto fpsTimer = lastFrame;
	int fpsCounter = 0;
	while (!glfwWindowShouldClose(window)) {
		auto currentFrame = std::chrono::steady_clock::now();
		float dt = std::chrono::duration<float>(currentFrame - lastFrame).count();
		float fpsDuration = std::chrono::duration<float>(currentFrame - fpsTimer).count();
		if (fpsDuration > 5.0f) {
			float fps = fpsCounter / fpsDuration;
			fpsCounter = 0;
			fpsTimer = currentFrame;
			std::cout << fps << std::endl;
		}
		++fpsCounter;
		lastFrame = currentFrame;

		update(dt);
		draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwTerminate();
	cleanupKinect();
	cleanup();
	cleanupVideo(&videoState);

	return 0;
}
