#pragma once

#include <stdint.h>

extern inline int DI(const int x, const int y);

typedef struct _Approx {
	float hori, vert, diag1, diag2;
} Approx;

Approx* approxes;

void approxMissingData(uint16_t* buf, int w, int h) {
	// Horizontal
	/*for (int y = 0; y < h; ++y) {
		int i0 = y * w;
		int i1 = (y + 1) * w - 1;
		int iPrev = -1;
		for (int i = i0; i <= i1; ++i) {
			approxes[i].hori = 0.0f;
			if (buf[i] == 0)
				continue;
			if (i > iPrev + 1) {
				int v = buf[i];
				int vPrev = v;
				float step = 0.0f;
				if (iPrev > -1) {
					vPrev = buf[iPrev];
					step = (float)(v - vPrev) / (float)(i - iPrev);
				}
				for (int j = iPrev + 1; j < i; ++j)
					approxes[j].hori = vPrev + step * (j - iPrev);
			}
			iPrev = i;
		}
		if (iPrev > -1 && iPrev < i1) {
			for (int i = iPrev + 1; i <= i1; ++i)
				approxes[i].hori = buf[iPrev];
		}
	}*/
	for (int y = 0; y < h; ++y) {
		int xPrev = -1;
		for (int x = 0; x < w; ++x) {
			approxes[DI(x, y)].hori = 0.0f;
			if (buf[DI(x, y)] == 0)
				continue;
			if (x > xPrev + 1) {
				int v = buf[DI(x, y)];
				int vPrev = v;
				float step = 0.0f;
				if (xPrev > -1) {
					vPrev = buf[DI(xPrev, y)];
					step = (float)(v - vPrev) / (float)(x - xPrev);
				}
				for (int x2 = xPrev + 1; x2 < x; ++x2)
					approxes[DI(x2, y)].hori = vPrev + step * (x2 - xPrev);
			}
			xPrev = x;
		}
		if (xPrev > -1 && xPrev < w - 1) {
			for (int x = xPrev + 1; x < w; ++x)
				approxes[DI(x, y)].hori = buf[DI(xPrev, y)];
		}
	}
	// Vertical
	for (int x = 0; x < w; ++x) {
		int yPrev = -1;
		for (int y = 0; y < h; ++y) {
			approxes[DI(x, y)].vert = 0.0f;
			if (buf[DI(x, y)] == 0)
				continue;
			if (y > yPrev + 1) {
				int v = buf[DI(x, y)];
				int vPrev = v;
				float step = 0.0f;
				if (yPrev > -1) {
					vPrev = buf[DI(x, yPrev)];
					step = (float)(v - vPrev) / (float)(y - yPrev);
				}
				for (int y2 = yPrev + 1; y2 < y; ++y2)
					approxes[DI(x, y2)].vert = vPrev + step * (y2 - yPrev);
			}
			yPrev = y;
		}
		if (yPrev > -1 && yPrev < h - 1) {
			for (int y = yPrev + 1; y < h; ++y)
				approxes[DI(x, y)].vert = buf[DI(x, yPrev)];
		}
	}

	// Apply approxes
	for (int i = 0; i < w * h; ++i) {
		if (buf[i] == 0) {
			float sum = 0.0;
			int count = 0;
			if (approxes[i].hori > 0) { sum += approxes[i].hori; ++count; }
			if (approxes[i].vert > 0) { sum += approxes[i].vert; ++count; }
			//if (approxes[i].diag1 > 0) { sum += approxes[i].diag1; ++count; }
			//if (approxes[i].diag2 > 0) { sum += approxes[i].diag2; ++count; }
			buf[i] = (uint16_t)(sum / (float)count); // TODO: Round approximation
		}
	}
}