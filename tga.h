#ifndef TGA_H
#define TGA_H

#include <vector>

struct TgaImage
{
	int width;
	int height;
	std::vector<unsigned char> rgba;
};

bool saveTGA(const char* path, const TgaImage& image);
bool loadTGA(const char* path, TgaImage& image);
unsigned uploadTexture(const TgaImage& image);

TgaImage makeSoftDisc(unsigned char r, unsigned char g, unsigned char b);
TgaImage makeTreeImage();
TgaImage makeSkyFace(unsigned char r0, unsigned char g0, unsigned char b0,
	unsigned char r1, unsigned char g1, unsigned char b1, bool clouds);
TgaImage makeEnvironmentImage();

#endif
