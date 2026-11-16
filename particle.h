#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec.h"

enum ParticleKind
{
	KIND_FIRE = 0,
	KIND_SNOW,
	KIND_WATER,
	KIND_CABLE,
	KIND_CLOTH,
	KIND_FUR,
	KIND_BOID
};

enum
{
	EMIT_CLAW = 0,
	EMIT_PIPE = 1,
	EMIT_ARM = 2,
	EMIT_COUNT = 3
};

struct Particle
{
	Vec3f pos;
	Vec3f vel;
	Vec3f color;
	float mass;
	float life;
	float age;
	float size;
	float radius;
	int kind;
	int pinId;
	bool alive;

	Particle()
		: pos(0, 0, 0), vel(0, 0, 0), color(1, 1, 1),
		  mass(1.0f), life(10.0f), age(0.0f), size(0.12f), radius(0.05f),
		  kind(KIND_WATER), pinId(-1), alive(false) {}
};

#endif
