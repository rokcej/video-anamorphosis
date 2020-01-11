#pragma once

#include <stdint.h>

extern inline int DI(const int x, const int y);

typedef struct _Approx {
	float hori, vert, diag1, diag2;
} Approx;

Approx* approxes;

void approxMissingData(uint16_t* buf, int w, int h) {
	// Horizontal
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
	
	// Diagonal
	for (int count = 0; count < w + h - 1; ++count) {
		{ // Diagonal 1 (top left -> bottom right)
			int dx = count, dy = 0;
			if (count >= w) { dx = 0; dy = count - w + 1; }
			int dxPrev = dx - 1, dyPrev = dy - 1;
			while (dx < w && dy < h) {
				approxes[DI(dx, dy)].diag1 = 0.0f;
				if (buf[DI(dx, dy)] > 0) {
					if (dx > dxPrev + 1) {
						int v = buf[DI(dx, dy)];
						int vPrev = v;
						float step = 0.0f;
						if (dxPrev > -1 && dyPrev > -1) {
							vPrev = buf[DI(dxPrev, dyPrev)];
							step = (float)(v - vPrev) / (float)(dx - dxPrev);
						}
						int dx2 = dxPrev + 1, dy2 = dyPrev + 1;
						while (dx2 < dx) {
							approxes[DI(dx2, dy2)].diag1 = vPrev + step * (dx2 - dxPrev);
							++dx2; ++dy2;
						}
					}
					dxPrev = dx; dyPrev = dy;
				}
				++dx; ++dy;
			}
			if (dxPrev > -1 && dxPrev < w - 1 && dyPrev > -1 && dyPrev < h - 1) {
				dx = dxPrev + 1; dy = dyPrev + 1;
				while (dx < w && dy < h) {
					approxes[DI(dx, dy)].diag1 = buf[DI(dxPrev, dyPrev)];
					++dx; ++dy;
				}
			}
		}
		{ // Diagonal 2 (top right -> bottom left)
			int dx = count, dy = 0;
			if (count >= w) { dx = w - 1; dy = count - w + 1; }
			int dxPrev = dx + 1, dyPrev = dy - 1;
			while (dx >= 0 && dy < h) {
				approxes[DI(dx, dy)].diag2 = 0.0f;
				if (buf[DI(dx, dy)] > 0) {
					if (dx < dxPrev - 1) {
						int v = buf[DI(dx, dy)];
						int vPrev = v;
						float step = 0.0f;
						if (dxPrev < w && dyPrev > -1) {
							vPrev = buf[DI(dxPrev, dyPrev)];
							step = (float)(v - vPrev) / (float)(dxPrev - dx);
						}
						int dx2 = dxPrev - 1, dy2 = dyPrev + 1;
						while (dx2 > dx) {
							approxes[DI(dx2, dy2)].diag2 = vPrev + step * (dxPrev - dx2);
							--dx2; ++dy2;
						}
					}
					dxPrev = dx; dyPrev = dy;
				}
				--dx; ++dy;
			}
			if (dxPrev < w && dxPrev > 0 && dyPrev > -1 && dyPrev < h - 1) {
				dx = dxPrev - 1; dy = dyPrev + 1;
				while (dx >= 0 && dy < h) {
					approxes[DI(dx, dy)].diag2 = buf[DI(dxPrev, dyPrev)];
					--dx; ++dy;
				}
			}
		}
	}

	// Apply approxes
	for (int i = 0; i < w * h; ++i) {
		if (buf[i] == 0) {
			float sum = 0.0;
			int count = 0;
			if (approxes[i].hori > 0) { sum += approxes[i].hori; ++count; }
			if (approxes[i].vert > 0) { sum += approxes[i].vert; ++count; }
			if (approxes[i].diag1 > 0) { sum += approxes[i].diag1; ++count; }
			if (approxes[i].diag2 > 0) { sum += approxes[i].diag2; ++count; }
			buf[i] = (uint16_t)(sum / (float)count); // TODO: Round approximation
		}
	}
}
