#include "tga.h"

#include <FL/gl.h>
#include <cstdio>
#include <cmath>
#include <cstdlib>

namespace {

void put16(FILE* f, int v)
{
	unsigned char b[2];
	b[0] = (unsigned char)(v & 255);
	b[1] = (unsigned char)((v >> 8) & 255);
	fwrite(b, 1, 2, f);
}

int get16(FILE* f)
{
	unsigned char b[2];
	if (fread(b, 1, 2, f) != 2)
		return -1;
	return b[0] | (b[1] << 8);
}

}

bool saveTGA(const char* path, const TgaImage& image)
{
	FILE* f = fopen(path, "wb");
	if (!f)
		return false;
	unsigned char header[18];
	for (int i = 0; i < 18; ++i)
		header[i] = 0;
	header[2] = 2;
	header[16] = 32;
	header[17] = 8 | 32; // 8 bits of alpha, origin at top
	header[12] = (unsigned char)(image.width & 255);
	header[13] = (unsigned char)((image.width >> 8) & 255);
	header[14] = (unsigned char)(image.height & 255);
	header[15] = (unsigned char)((image.height >> 8) & 255);
	fwrite(header, 1, 18, f);
	for (int i = 0; i < image.width * image.height; ++i) {
		unsigned char bgra[4];
		bgra[0] = image.rgba[i * 4 + 2];
		bgra[1] = image.rgba[i * 4 + 1];
		bgra[2] = image.rgba[i * 4 + 0];
		bgra[3] = image.rgba[i * 4 + 3];
		fwrite(bgra, 1, 4, f);
	}
	fclose(f);
	return true;
}

bool loadTGA(const char* path, TgaImage& image)
{
	FILE* f = fopen(path, "rb");
	if (!f)
		return false;
	unsigned char idLength = 0, colorMapType = 0, imageType = 0;
	if (fread(&idLength, 1, 1, f) != 1) { fclose(f); return false; }
	if (fread(&colorMapType, 1, 1, f) != 1) { fclose(f); return false; }
	if (fread(&imageType, 1, 1, f) != 1) { fclose(f); return false; }
	unsigned char skip[5];
	if (fread(skip, 1, 5, f) != 5) { fclose(f); return false; }
	if (get16(f) < 0 || get16(f) < 0) { fclose(f); return false; }
	int width = get16(f);
	int height = get16(f);
	int bpp = fgetc(f);
	int descriptor = fgetc(f);
	if (width <= 0 || height <= 0 || (imageType != 2 && imageType != 3)) {
		fclose(f);
		return false;
	}
	if (idLength)
		fseek(f, idLength, SEEK_CUR);
	int channels = bpp / 8;
	if (channels < 3 || channels > 4) {
		fclose(f);
		return false;
	}
	bool topOrigin = (descriptor & 32) != 0;
	image.width = width;
	image.height = height;
	image.rgba.assign(width * height * 4, 255);
	std::vector<unsigned char> row(width * channels);
	for (int y = 0; y < height; ++y) {
		if (fread(&row[0], 1, row.size(), f) != row.size()) {
			fclose(f);
			return false;
		}
		int destY = topOrigin ? y : (height - 1 - y);
		for (int x = 0; x < width; ++x) {
			int di = (destY * width + x) * 4;
			int si = x * channels;
			image.rgba[di + 2] = row[si + 0];
			image.rgba[di + 1] = row[si + 1];
			image.rgba[di + 0] = row[si + 2];
			image.rgba[di + 3] = (channels == 4) ? row[si + 3] : 255;
		}
	}
	fclose(f);
	return true;
}

unsigned uploadTexture(const TgaImage& image)
{
	GLuint tex = 0;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width, image.height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, &image.rgba[0]);
	return tex;
}

TgaImage makeSoftDisc(unsigned char r, unsigned char g, unsigned char b)
{
	const int n = 64;
	TgaImage img;
	img.width = n;
	img.height = n;
	img.rgba.assign(n * n * 4, 0);
	for (int y = 0; y < n; ++y) {
		for (int x = 0; x < n; ++x) {
			float dx = (x + 0.5f) / n * 2.0f - 1.0f;
			float dy = (y + 0.5f) / n * 2.0f - 1.0f;
			float d = sqrtf(dx * dx + dy * dy);
			float a = 1.0f - d;
			if (a < 0.0f) a = 0.0f;
			a = a * a;
			int i = (y * n + x) * 4;
			img.rgba[i + 0] = r;
			img.rgba[i + 1] = g;
			img.rgba[i + 2] = b;
			img.rgba[i + 3] = (unsigned char)(a * 255.0f);
		}
	}
	return img;
}

TgaImage makeTreeImage()
{
	const int n = 64;
	TgaImage img;
	img.width = n;
	img.height = n;
	img.rgba.assign(n * n * 4, 0);
	for (int y = 0; y < n; ++y) {
		for (int x = 0; x < n; ++x) {
			int i = (y * n + x) * 4;
			bool trunk = x > 28 && x < 36 && y > 30;
			float dx = (x - 32) / 22.0f;
			float dy = (y - 24) / 18.0f;
			bool canopy = dx * dx + dy * dy < 1.0f && y < 46;
			if (trunk) {
				img.rgba[i + 0] = 92;
				img.rgba[i + 1] = 58;
				img.rgba[i + 2] = 28;
				img.rgba[i + 3] = 255;
			}
			else if (canopy) {
				float shade = 0.75f + 0.25f * (1.0f - dy);
				img.rgba[i + 0] = (unsigned char)(30 * shade);
				img.rgba[i + 1] = (unsigned char)(140 * shade);
				img.rgba[i + 2] = (unsigned char)(45 * shade);
				img.rgba[i + 3] = 255;
			}
		}
	}
	return img;
}

TgaImage makeSkyFace(unsigned char r0, unsigned char g0, unsigned char b0,
	unsigned char r1, unsigned char g1, unsigned char b1, bool clouds)
{
	const int n = 32;
	TgaImage img;
	img.width = n;
	img.height = n;
	img.rgba.assign(n * n * 4, 255);
	for (int y = 0; y < n; ++y) {
		float t = y / (float)(n - 1);
		unsigned char r = (unsigned char)(r0 + (r1 - r0) * t);
		unsigned char g = (unsigned char)(g0 + (g1 - g0) * t);
		unsigned char b = (unsigned char)(b0 + (b1 - b0) * t);
		for (int x = 0; x < n; ++x) {
			int i = (y * n + x) * 4;
			img.rgba[i] = r;
			img.rgba[i + 1] = g;
			img.rgba[i + 2] = b;
			if (clouds) {
				float cx = (x - 10) / 6.0f;
				float cy = (y - 20) / 3.0f;
				if (cx * cx + cy * cy < 1.0f) {
					img.rgba[i] = 245;
					img.rgba[i + 1] = 245;
					img.rgba[i + 2] = 250;
				}
			}
		}
	}
	return img;
}

TgaImage makeEnvironmentImage()
{
	const int n = 64;
	TgaImage img;
	img.width = n;
	img.height = n;
	img.rgba.assign(n * n * 4, 255);
	for (int y = 0; y < n; ++y) {
		for (int x = 0; x < n; ++x) {
			int i = (y * n + x) * 4;
			if (y < n / 2) {
				img.rgba[i] = 70;
				img.rgba[i + 1] = 120;
				img.rgba[i + 2] = 190;
			}
			else {
				img.rgba[i] = 90;
				img.rgba[i + 1] = 80;
				img.rgba[i + 2] = 50;
			}
			float dx = (x - 20) / 8.0f;
			float dy = (y - 16) / 4.0f;
			if (dx * dx + dy * dy < 1.0f) {
				img.rgba[i] = 250;
				img.rgba[i + 1] = 250;
				img.rgba[i + 2] = 255;
			}
			dx = (x - 44) / 10.0f;
			dy = (y - 20) / 5.0f;
			if (dx * dx + dy * dy < 1.0f) {
				img.rgba[i] = 230;
				img.rgba[i + 1] = 235;
				img.rgba[i + 2] = 245;
			}
		}
	}
	return img;
}
