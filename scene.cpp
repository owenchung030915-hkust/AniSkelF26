#include "scene.h"
#include "tga.h"
#include "modelerdraw.h"

#include <FL/gl.h>
#include <cstdio>

namespace {

unsigned texOrZero(const TgaImage& image)
{
	if (image.rgba.empty())
		return 0;
	return uploadTexture(image);
}

unsigned skyTex(int face)
{
	static unsigned tex[6] = { 0, 0, 0, 0, 0, 0 };
	if (!tex[0]) {
		tex[0] = texOrZero(makeSkyFace(90, 150, 220, 180, 210, 245, true));
		tex[1] = texOrZero(makeSkyFace(80, 140, 210, 170, 200, 235, false));
		tex[2] = texOrZero(makeSkyFace(70, 130, 200, 160, 190, 230, true));
		tex[3] = texOrZero(makeSkyFace(100, 160, 220, 190, 215, 240, false));
		tex[4] = texOrZero(makeSkyFace(120, 175, 230, 200, 225, 250, false));
		tex[5] = texOrZero(makeSkyFace(70, 90, 50, 40, 70, 40, false));
	}
	return tex[face];
}

unsigned treeTexture()
{
	static unsigned tex = 0;
	static bool tried = false;
	if (tried)
		return tex;
	tried = true;
	TgaImage authored = makeTreeImage();
	const char* path = "tree.tga";
	if (saveTGA(path, authored)) {
		TgaImage loaded;
		if (loadTGA(path, loaded) && loaded.width == authored.width) {
			printf("Transparent TGA: wrote and reloaded %s (%dx%d, 32-bit)\n",
				path, loaded.width, loaded.height);
			tex = uploadTexture(loaded);
			return tex;
		}
	}
	printf("Transparent TGA: using generated tree image\n");
	tex = uploadTexture(authored);
	return tex;
}

unsigned envTexture()
{
	static unsigned tex = 0;
	if (!tex)
		tex = uploadTexture(makeEnvironmentImage());
	return tex;
}

void skyQuad(unsigned tex, float x0, float y0, float z0, float x1, float y1, float z1,
	float x2, float y2, float z2, float x3, float y3, float z3)
{
	glBindTexture(GL_TEXTURE_2D, tex);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); glVertex3f(x0, y0, z0);
	glTexCoord2f(1, 0); glVertex3f(x1, y1, z1);
	glTexCoord2f(1, 1); glVertex3f(x2, y2, z2);
	glTexCoord2f(0, 1); glVertex3f(x3, y3, z3);
	glEnd();
}

}

void drawSkybox()
{
	GLfloat m[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, m);
	m[12] = m[13] = m[14] = 0.0f;

	glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT);
	glPushMatrix();
	glLoadMatrixf(m);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glEnable(GL_TEXTURE_2D);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glColor3f(1, 1, 1);

	const float s = 40.0f;
	skyQuad(skyTex(0), -s, -s, -s,  s, -s, -s,  s,  s, -s, -s,  s, -s);
	skyQuad(skyTex(1),  s, -s,  s, -s, -s,  s, -s,  s,  s,  s,  s,  s);
	skyQuad(skyTex(2), -s, -s,  s, -s, -s, -s, -s,  s, -s, -s,  s,  s);
	skyQuad(skyTex(3),  s, -s, -s,  s, -s,  s,  s,  s,  s,  s,  s, -s);
	skyQuad(skyTex(4), -s,  s, -s,  s,  s, -s,  s,  s,  s, -s,  s,  s);
	skyQuad(skyTex(5), -s, -s,  s,  s, -s,  s,  s, -s, -s, -s, -s, -s);

	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
	glPopAttrib();
}

void drawHeightfield()
{
	static GLuint list = 0;
	if (list == 0) {
		const int n = 48;
		const float extent = 12.0f;
		list = glGenLists(1);
		glNewList(list, GL_COMPILE);
		for (int z = 0; z < n - 1; ++z) {
			glBegin(GL_TRIANGLE_STRIP);
			for (int x = 0; x < n; ++x) {
				for (int row = 1; row >= 0; --row) {
					float fx = -extent + 2.0f * extent * (x / (float)(n - 1));
					float fz = -extent + 2.0f * extent * ((z + row) / (float)(n - 1));
					float y = terrainHeight(fx, fz);
					float yx = terrainHeight(fx + 0.15f, fz);
					float yz = terrainHeight(fx, fz + 0.15f);
					float nx = y - yx;
					float nz = y - yz;
					float ny = 0.15f;
					float len = sqrtf(nx * nx + ny * ny + nz * nz);
					glNormal3f(nx / len, ny / len, nz / len);
					float shade = 0.35f + 0.65f * (y + 0.3f);
					if (shade < 0.2f) shade = 0.2f;
					if (shade > 1.0f) shade = 1.0f;
					glColor3f(0.25f * shade, 0.45f * shade, 0.18f * shade);
					glVertex3f(fx, y, fz);
				}
			}
			glEnd();
		}
		glEndList();
		printf("Display list %u compiled for the heightfield (%d x %d)\n", list, n, n);
	}

	glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	setDiffuseColor(0.3f, 0.55f, 0.25f);
	setAmbientColor(0.15f, 0.25f, 0.12f);
	glCallList(list);
	glPopAttrib();
}

void drawTrees()
{
	unsigned tex = treeTexture();
	if (!tex)
		return;
	glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_ALPHA_TEST);
	glAlphaFunc(GL_GREATER, 0.3f);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_LIGHTING);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glColor4f(1, 1, 1, 1);

	const float spots[3][2] = { { 6.0f, 3.0f }, { -4.0f, 6.5f }, { 5.0f, -5.0f } };
	for (int i = 0; i < 3; ++i) {
		float x = spots[i][0];
		float z = spots[i][1];
		float y = terrainHeight(x, z);
		for (int cross = 0; cross < 2; ++cross) {
			glPushMatrix();
			glTranslatef(x, y, z);
			glRotatef(cross ? 90.0f : 0.0f, 0, 1, 0);
			glBegin(GL_QUADS);
			glTexCoord2f(0, 0); glVertex3f(-0.7f, 0, 0);
			glTexCoord2f(1, 0); glVertex3f(0.7f, 0, 0);
			glTexCoord2f(1, 1); glVertex3f(0.7f, 2.4f, 0);
			glTexCoord2f(0, 1); glVertex3f(-0.7f, 2.4f, 0);
			glEnd();
			glPopMatrix();
		}
	}
	glPopAttrib();
}

void drawChromeSphere()
{
	unsigned tex = envTexture();
	glPushMatrix();
	glTranslatef(4.4f, terrainHeight(4.4f, 1.6f) + 0.75f, 1.6f);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
	glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	setDiffuseColor(1, 1, 1);
	setAmbientColor(0.4f, 0.4f, 0.4f);
	drawSphere(0.7);
	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}
