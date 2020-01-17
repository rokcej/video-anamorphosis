// Functions for saving a .bmp image

#pragma once

#include <algorithm>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define FILE_HEADER_SIZE 14
#define INFO_HEADER_SIZE 40

void bmpSaveImage(float* data, int width, int height, const char* fileName);
static void createFileHeader(int width, int height, unsigned char* fileHeader);
static void createInfoHeader(int width, int height, unsigned char* infoHeader);

// data = { r, g, b, ... }
void bmpSaveImage(float* data, int width, int height, const char* fileName) {
	// Get headers
	unsigned char fileHeader[FILE_HEADER_SIZE];
	unsigned char infoHeader[INFO_HEADER_SIZE];
	createFileHeader(width, height, fileHeader);
	createInfoHeader(width, height, infoHeader);

	// Padding
	unsigned char padding[3] = { 0, 0, 0 };
	int paddingSize = (4 - (width * 3) % 4) % 4;

	// Write to file
	FILE* f = fopen(fileName, "wb");
	fwrite(fileHeader, 1, FILE_HEADER_SIZE, f);
	fwrite(infoHeader, 1, INFO_HEADER_SIZE, f);
	for (int y = height - 1; y >= 0; --y) {
		for (int x = 0; x < width; ++x) {
			int i = (y * width + x) * 3;
			unsigned char r = (unsigned char)round(data[i] * 255.0);
			unsigned char g = (unsigned char)round(data[i] * 255.0);
			unsigned char b = (unsigned char)round(data[i] * 255.0);
			fwrite(&b, 1, 1, f);
			fwrite(&g, 1, 1, f);
			fwrite(&r, 1, 1, f);

			fwrite(padding, 1, paddingSize, f);
		}
	}
	fclose(f);
}

// data = { grayscale, ... }
void bmpSaveKinectDepth(uint16_t* data, int width, int height, const char* fileName) {
	// Get headers
	unsigned char fileHeader[FILE_HEADER_SIZE];
	unsigned char infoHeader[INFO_HEADER_SIZE];
	createFileHeader(width, height, fileHeader);
	createInfoHeader(width, height, infoHeader);

	// Padding
	unsigned char padding[3] = { 0, 0, 0 };
	int paddingSize = (4 - (width * 3) % 4) % 4;

	// Get max value
	uint16_t maxDist = 500; // Minimum kinect distance, 0.5m
	uint16_t minDist = 65535; // Maximum 16bit value
	for (int i = 1; i < width * height; ++i) {
		if (data[i] > maxDist)
			maxDist = data[i];
		else if (data[i] >= 500 && data[i] < minDist)
			minDist = data[i];
	}
	float divisor = (float)(maxDist - minDist);

	// Write to file
	FILE* f = fopen(fileName, "wb");
	fwrite(fileHeader, 1, FILE_HEADER_SIZE, f);
	fwrite(infoHeader, 1, INFO_HEADER_SIZE, f);
	for (int y = height - 1; y >= 0; --y) {
		for (int x = 0; x < width; ++x) {
			uint16_t dist = std::max(data[y * width + x], minDist);
			unsigned char a = (unsigned char)roundf((float)(dist - minDist) * 255.0f / (float)divisor);
			fwrite(&a, 1, 1, f);
			fwrite(&a, 1, 1, f);
			fwrite(&a, 1, 1, f);

			fwrite(padding, 1, paddingSize, f);
		}
	}
	fclose(f);
}

static void createFileHeader(int width, int height, unsigned char* fileHeader) {
	int dataOffset = FILE_HEADER_SIZE + INFO_HEADER_SIZE;
	int fileSize = dataOffset + 3 * height * width;

	memset(fileHeader, 0, FILE_HEADER_SIZE);
	// bfType
	fileHeader[0] = 'B';
	fileHeader[1] = 'M';
	// bfSize
	fileHeader[2] = (fileSize) & 0xFF;
	fileHeader[3] = (fileSize >> 8) & 0xFF;
	fileHeader[4] = (fileSize >> 16) & 0xFF;
	fileHeader[5] = (fileSize >> 24) & 0xFF;
	// bfOffBits
	fileHeader[10] = dataOffset;
}

static void createInfoHeader(int width, int height, unsigned char* infoHeader) {
	memset(infoHeader, 0, INFO_HEADER_SIZE);
	// biSize
	infoHeader[0] = INFO_HEADER_SIZE;
	// biWidth
	infoHeader[4] = (width) & 0xFF;
	infoHeader[5] = (width >> 8) & 0xFF;
	infoHeader[6] = (width >> 16) & 0xFF;
	infoHeader[7] = (width >> 24) & 0xFF;
	// biHeight
	infoHeader[8] = (height) & 0xFF;
	infoHeader[9] = (height >> 8) & 0xFF;
	infoHeader[10] = (height >> 16) & 0xFF;
	infoHeader[11] = (height >> 24) & 0xFF;
	// biPlanes
	infoHeader[12] = 1;
	// biCount
	infoHeader[14] = 24;
}
