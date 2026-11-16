#ifndef SCENE_H
#define SCENE_H

#include <cmath>

inline float terrainHeight(float x, float z)
{
	return 0.18f * sinf(x * 0.45f) * cosf(z * 0.38f)
		+ 0.05f * sinf((x + z) * 0.7f);
}

void drawSkybox();
void drawHeightfield();
void drawTrees();
void drawChromeSphere();

#endif
