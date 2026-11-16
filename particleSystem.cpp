#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerapp.h"
#include "animcontrols.h"
#include "scene.h"
#include "tga.h"

#include <FL/gl.h>
#include <cmath>
#include <cstdlib>
#include <cstdio>

namespace {

const float kGravity = 9.8f;

float frand(float a, float b)
{
	return a + (b - a) * (rand() / (float)RAND_MAX);
}

int frameIndex(float t, float fps)
{
	if (fps < 1.0f) fps = 30.0f;
	return (int)floorf(t * fps + 0.5f);
}

Mat4f readModelView()
{
	GLfloat m[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, m);
	Mat4f mat(m[0], m[1], m[2], m[3],
		m[4], m[5], m[6], m[7],
		m[8], m[9], m[10], m[11],
		m[12], m[13], m[14], m[15]);
	return mat.transpose();
}

Mat4f rigidInverse(const Mat4f& m)
{
	Mat4f r;
	r[0][0] = m[0][0]; r[0][1] = m[1][0]; r[0][2] = m[2][0]; r[0][3] = 0;
	r[1][0] = m[0][1]; r[1][1] = m[1][1]; r[1][2] = m[2][1]; r[1][3] = 0;
	r[2][0] = m[0][2]; r[2][1] = m[1][2]; r[2][2] = m[2][2]; r[2][3] = 0;
	r[3][0] = 0; r[3][1] = 0; r[3][2] = 0; r[3][3] = 1;
	float tx = m[0][3], ty = m[1][3], tz = m[2][3];
	r[0][3] = -(r[0][0] * tx + r[0][1] * ty + r[0][2] * tz);
	r[1][3] = -(r[1][0] * tx + r[1][1] * ty + r[1][2] * tz);
	r[2][3] = -(r[2][0] * tx + r[2][1] * ty + r[2][2] * tz);
	return r;
}

void safeNormalize(Vec3f& v)
{
	float len = (float)v.length();
	if (len > 1e-6f)
		v /= len;
}

unsigned gFireTex = 0, gSnowTex = 0, gWaterTex = 0;

void ensureParticleTextures()
{
	if (gFireTex)
		return;
	gFireTex = uploadTexture(makeSoftDisc(255, 220, 120));
	gSnowTex = uploadTexture(makeSoftDisc(245, 248, 255));
	gWaterTex = uploadTexture(makeSoftDisc(140, 190, 255));
}

}

ParticleSystem::ParticleSystem()
{
	bake_fps = 30.0f;
	bake_start_time = 0.0f;
	bake_end_time = 0.0f;
	baked = false;
	simulate = false;
	dirty = false;
	m_viewReady = false;
	m_structuresReady = false;
	m_hasTime = false;
	m_lastTime = 0.0f;
	m_fireAcc = m_snowAcc = m_waterAcc = 0.0f;
	m_clothStart = m_cableStart = m_furStart = m_boidStart = 0;
	m_clothCols = 8;
	m_clothRows = 5;
	m_cableCount = 8;
	m_furStrands = 8;
	m_furLen = 3;
	m_boidCount = 16;
}

ParticleSystem::~ParticleSystem() {}

void ParticleSystem::setEmitter(int emitterId, float dirX, float dirY, float dirZ)
{
	if (!m_viewReady || emitterId < 0 || emitterId >= EMIT_COUNT)
		return;
	Mat4f world = m_invView * readModelView();
	Vec3f origin = world * Vec3f(0, 0, 0);
	Vec3f dir = (world * Vec3f(dirX, dirY, dirZ)) - origin;
	safeNormalize(dir);
	float t = ModelerApplication::Instance()->GetTime();
	Emitter& e = m_emit[emitterId];
	if (e.valid && t > e.lastT + 1e-5f)
		e.vel = (origin - e.pos) / (t - e.lastT);
	e.pos = origin;
	e.dir = dir;
	e.lastT = t;
	e.valid = true;
}

void ParticleSystem::setArmPins(float armHeight)
{
	if (!m_viewReady)
		return;
	Mat4f world = m_invView * readModelView();
	Emitter& e = m_emit[EMIT_ARM];
	e.pins.clear();
	e.furPins.clear();
	e.furDirs.clear();
	float h = armHeight < 0.4f ? 0.4f : armHeight;
	for (int c = 0; c < m_clothCols; ++c) {
		float z = -0.22f + 0.44f * (c / (float)(m_clothCols - 1));
		e.pins.push_back(world * Vec3f(0.32f, h * 0.82f, z));
	}
	for (int s = 0; s < m_furStrands; ++s) {
		float y = 0.15f + (h - 0.3f) * (s / (float)(m_furStrands - 1));
		e.furPins.push_back(world * Vec3f(-0.30f, y, 0.0f));
		Vec3f dir = (world * Vec3f(-1.0f, 0.15f, (s % 2) ? 0.25f : -0.25f)) - (world * Vec3f(0, 0, 0));
		safeNormalize(dir);
		e.furDirs.push_back(dir);
	}
	float t = ModelerApplication::Instance()->GetTime();
	Vec3f origin = world * Vec3f(0, h * 0.5f, 0);
	if (e.valid && t > e.lastT + 1e-5f)
		e.vel = (origin - e.pos) / (t - e.lastT);
	e.pos = origin;
	e.dir = (world * Vec3f(0, 1, 0)) - (world * Vec3f(0, 0, 0));
	safeNormalize(e.dir);
	e.lastT = t;
	e.valid = true;
}

void ParticleSystem::startSimulation(float t)
{
	m_particles.clear();
	m_springs.clear();
	m_bake.clear();
	m_structuresReady = false;
	m_hasTime = false;
	m_lastTime = t;
	m_fireAcc = m_snowAcc = m_waterAcc = 0.0f;
	baked = false;
	bake_fps = (float)ModelerApplication::Instance()->GetFps();
	if (bake_fps < 1.0f) bake_fps = 30.0f;
	bake_start_time = t;
	bake_end_time = -1;
	simulate = true;
	dirty = true;
}

void ParticleSystem::stopSimulation(float t)
{
	bake_end_time = t;
	simulate = false;
	dirty = true;
	baked = !m_bake.empty();
}

void ParticleSystem::resetSimulation(float t)
{
	(void)t;
	clearBaked();
	m_particles.clear();
	m_springs.clear();
	m_structuresReady = false;
	simulate = false;
	dirty = true;
}

void ParticleSystem::clearBaked()
{
	m_bake.clear();
	baked = false;
	bake_start_time = 0;
	bake_end_time = 0;
	m_hasTime = false;
}

void ParticleSystem::addSpring(int a, int b, float ks, float kd)
{
	Spring s;
	s.a = a;
	s.b = b;
	s.ks = ks;
	s.kd = kd;
	Vec3f d = m_particles[a].pos - m_particles[b].pos;
	s.rest = (float)d.length();
	if (s.rest < 1e-3f) s.rest = 1e-3f;
	m_springs.push_back(s);
}

void ParticleSystem::buildStructures()
{
	m_particles.clear();
	m_springs.clear();

	m_cableStart = 0;
	m_cableCount = 8;
	for (int i = 0; i < m_cableCount; ++i) {
		Particle p;
		p.alive = true;
		p.kind = KIND_CABLE;
		p.mass = 0.12f;
		p.radius = 0.04f;
		p.size = 0.05f;
		p.color = Vec3f(0.75f, 0.75f, 0.8f);
		p.pos = m_emit[EMIT_CLAW].pos + Vec3f(0, -0.16f * i, 0);
		p.pinId = (i == 0) ? 0 : -1;
		m_particles.push_back(p);
	}
	for (int i = 1; i < m_cableCount; ++i)
		addSpring(m_cableStart + i - 1, m_cableStart + i, 48.0f, 6.0f);

	m_clothStart = (int)m_particles.size();
	m_clothCols = 8;
	m_clothRows = 5;
	for (int r = 0; r < m_clothRows; ++r) {
		for (int c = 0; c < m_clothCols; ++c) {
			Particle p;
			p.alive = true;
			p.kind = KIND_CLOTH;
			p.mass = 0.08f;
			p.radius = 0.03f;
			p.color = Vec3f(0.85f, 0.25f, 0.2f);
			p.pos = m_emit[EMIT_ARM].pins[c] + Vec3f(0.05f * r, -0.16f * r, 0);
			p.pinId = (r == 0) ? c : -1;
			m_particles.push_back(p);
		}
	}
	for (int r = 0; r < m_clothRows; ++r) {
		for (int c = 0; c < m_clothCols; ++c) {
			int i = m_clothStart + r * m_clothCols + c;
			if (c + 1 < m_clothCols)
				addSpring(i, i + 1, 36.0f, 4.5f);
			if (r + 1 < m_clothRows)
				addSpring(i, i + m_clothCols, 36.0f, 4.5f);
			if (c + 1 < m_clothCols && r + 1 < m_clothRows) {
				addSpring(i, i + m_clothCols + 1, 24.0f, 3.0f);
				addSpring(i + 1, i + m_clothCols, 24.0f, 3.0f);
			}
			if (c + 2 < m_clothCols)
				addSpring(i, i + 2, 14.0f, 2.0f);
			if (r + 2 < m_clothRows)
				addSpring(i, i + 2 * m_clothCols, 14.0f, 2.0f);
		}
	}

	m_furStart = (int)m_particles.size();
	m_furStrands = 8;
	m_furLen = 3;
	for (int s = 0; s < m_furStrands; ++s) {
		for (int k = 0; k < m_furLen; ++k) {
			Particle p;
			p.alive = true;
			p.kind = KIND_FUR;
			p.mass = 0.04f;
			p.radius = 0.02f;
			p.color = Vec3f(0.45f, 0.32f, 0.18f);
			p.pos = m_emit[EMIT_ARM].furPins[s] + m_emit[EMIT_ARM].furDirs[s] * (0.12f * k);
			p.pinId = (k == 0) ? s : -1;
			m_particles.push_back(p);
		}
		int root = m_furStart + s * m_furLen;
		for (int k = 1; k < m_furLen; ++k)
			addSpring(root + k - 1, root + k, 45.0f, 1.5f);
	}

	m_boidStart = (int)m_particles.size();
	m_boidCount = 16;
	for (int i = 0; i < m_boidCount; ++i) {
		float a = (float)i / (float)m_boidCount * 6.2831853f;
		Particle p;
		p.alive = true;
		p.kind = KIND_BOID;
		p.mass = 1.0f;
		p.radius = 0.15f;
		p.size = 0.45f;
		p.color = Vec3f(0.95f, 0.85f, 0.15f);
		if (i % 2) p.color = Vec3f(0.3f, 0.85f, 0.35f);
		p.pos = Vec3f(cosf(a) * 4.0f, 6.0f, sinf(a) * 4.0f);
		p.vel = Vec3f(-sinf(a) * 2.0f, 0.1f, cosf(a) * 2.0f);
		p.life = 1.0e6f;
		m_particles.push_back(p);
	}
	m_structuresReady = true;
}

bool ParticleSystem::isPinned(const Particle& p) const
{
	return p.alive && p.pinId >= 0 &&
		(p.kind == KIND_CABLE || p.kind == KIND_CLOTH || p.kind == KIND_FUR);
}

Vec3f ParticleSystem::pinTarget(const Particle& p) const
{
	if (p.kind == KIND_CABLE && m_emit[EMIT_CLAW].valid)
		return m_emit[EMIT_CLAW].pos;
	if (p.kind == KIND_CLOTH && p.pinId >= 0 && p.pinId < (int)m_emit[EMIT_ARM].pins.size())
		return m_emit[EMIT_ARM].pins[p.pinId];
	if (p.kind == KIND_FUR && p.pinId >= 0 && p.pinId < (int)m_emit[EMIT_ARM].furPins.size())
		return m_emit[EMIT_ARM].furPins[p.pinId];
	return p.pos;
}

int ParticleSystem::acquire(int kind, int maxCount)
{
	int alive = 0;
	int freeSlot = -1;
	for (int i = 0; i < (int)m_particles.size(); ++i) {
		if (m_particles[i].kind != kind)
			continue;
		if (m_particles[i].alive)
			++alive;
		else if (freeSlot < 0)
			freeSlot = i;
	}
	if (freeSlot >= 0)
		return freeSlot;
	if (alive >= maxCount)
		return -1;
	m_particles.push_back(Particle());
	m_particles.back().kind = kind;
	return (int)m_particles.size() - 1;
}

void ParticleSystem::spawnEphemeral(float dt)
{
	float rate = 1.0f;
	ModelerApplication* app = ModelerApplication::Instance();
	if (app)
		rate = (float)app->GetControlValue(PARTICLE_COUNT);
	if (rate < 0.0f) rate = 0.0f;

	if (m_emit[EMIT_PIPE].valid) {
		m_fireAcc += dt * rate * 18.0f;
		while (m_fireAcc >= 1.0f) {
			m_fireAcc -= 1.0f;
			int i = acquire(KIND_FIRE, 180);
			if (i < 0) break;
			Particle& p = m_particles[i];
			p.alive = true;
			p.kind = KIND_FIRE;
			p.mass = 0.05f;
			p.age = 0;
			p.life = frand(0.6f, 1.3f);
			p.radius = 0.02f;
			p.size = frand(0.12f, 0.22f);
			p.color = Vec3f(1.0f, frand(0.45f, 0.85f), 0.15f);
			p.pinId = -1;
			p.pos = m_emit[EMIT_PIPE].pos + Vec3f(frand(-0.02f, 0.02f), 0, frand(-0.02f, 0.02f));
			p.vel = m_emit[EMIT_PIPE].dir * frand(1.6f, 2.8f) + m_emit[EMIT_PIPE].vel
				+ Vec3f(frand(-0.3f, 0.3f), frand(0.0f, 0.4f), frand(-0.3f, 0.3f));
		}
	}
	if (m_emit[EMIT_CLAW].valid) {
		m_waterAcc += dt * rate * 22.0f;
		while (m_waterAcc >= 1.0f) {
			m_waterAcc -= 1.0f;
			int i = acquire(KIND_WATER, 220);
			if (i < 0) break;
			Particle& p = m_particles[i];
			p.alive = true;
			p.kind = KIND_WATER;
			p.mass = 0.06f;
			p.age = 0;
			p.life = frand(1.6f, 2.6f);
			p.radius = 0.05f;
			p.size = frand(0.08f, 0.14f);
			p.color = Vec3f(0.45f, 0.7f, 1.0f);
			p.pinId = -1;
			p.pos = m_emit[EMIT_CLAW].pos;
			p.vel = m_emit[EMIT_CLAW].vel + m_emit[EMIT_CLAW].dir * frand(1.5f, 3.2f)
				+ Vec3f(frand(-0.4f, 0.4f), frand(0.0f, 0.5f), frand(-0.4f, 0.4f));
		}
	}
	m_snowAcc += dt * (4.0f + rate);
	while (m_snowAcc >= 1.0f) {
		m_snowAcc -= 1.0f;
		int i = acquire(KIND_SNOW, 140);
		if (i < 0) break;
		Particle& p = m_particles[i];
		p.alive = true;
		p.kind = KIND_SNOW;
		p.mass = 0.03f;
		p.age = 0;
		p.life = frand(4.0f, 7.0f);
		p.radius = 0.04f;
		p.size = frand(0.06f, 0.12f);
		p.color = Vec3f(0.95f, 0.97f, 1.0f);
		p.pinId = -1;
		p.pos = Vec3f(frand(-8.0f, 8.0f), frand(7.0f, 9.0f), frand(-8.0f, 8.0f));
		p.vel = Vec3f(frand(-0.2f, 0.2f), frand(-0.4f, -0.1f), frand(-0.2f, 0.2f));
	}
}

void ParticleSystem::computeAccel(const std::vector<Vec3f>& pos, const std::vector<Vec3f>& vel,
	std::vector<Vec3f>& accel, const Vec3f& wind) const
{
	int n = (int)m_particles.size();
	std::vector<Vec3f> force(n, Vec3f(0, 0, 0));
	for (int i = 0; i < n; ++i) {
		const Particle& p = m_particles[i];
		if (!p.alive || isPinned(p))
			continue;
		float mass = p.mass > 1e-4f ? p.mass : 1.0f;
		if (p.kind == KIND_FIRE) {
			force[i] += Vec3f(0, -1.5f * mass, 0);
			if (m_emit[EMIT_PIPE].valid)
				force[i] += m_emit[EMIT_PIPE].dir * (5.0f * mass);
			force[i] += wind * 0.12f;
			force[i] -= vel[i] * 0.8f;
		}
		else if (p.kind == KIND_SNOW) {
			force[i] += Vec3f(0, -1.8f * mass, 0);
			force[i] += wind * 0.04f;
			force[i] -= vel[i] * 0.4f;
		}
		else if (p.kind == KIND_WATER) {
			force[i] += Vec3f(0, -kGravity * mass, 0);
			float push = 1.0f - p.age / (p.life > 0.1f ? p.life : 0.1f);
			if (push < 0.0f) push = 0.0f;
			if (m_emit[EMIT_CLAW].valid)
				force[i] += m_emit[EMIT_CLAW].dir * (3.0f * mass * push);
			force[i] += wind * 0.02f;
			force[i] -= vel[i] * 0.15f;
		}
		else if (p.kind == KIND_BOID) {
			force[i] -= vel[i] * 0.35f;
		}
		else {
			force[i] += Vec3f(0, -kGravity * mass, 0);
			float windScale = (p.kind == KIND_CLOTH) ? 0.45f : (p.kind == KIND_FUR ? 0.2f : 0.05f);
			force[i] += wind * windScale;
			force[i] -= vel[i] * (p.kind == KIND_CLOTH ? 0.35f : 0.2f);
		}
		accel[i] = force[i] / mass;
	}

	for (size_t s = 0; s < m_springs.size(); ++s) {
		const Spring& sp = m_springs[s];
		if (!m_particles[sp.a].alive || !m_particles[sp.b].alive)
			continue;
		Vec3f d = pos[sp.b] - pos[sp.a];
		float len = (float)d.length();
		if (len < 1e-5f)
			continue;
		Vec3f nrm = d / len;
		float stretch = sp.ks * (len - sp.rest);
		float damp = sp.kd * ((vel[sp.b] - vel[sp.a]) * nrm);
		Vec3f f = nrm * (stretch + damp);
		if (!isPinned(m_particles[sp.a]))
			accel[sp.a] += f / m_particles[sp.a].mass;
		if (!isPinned(m_particles[sp.b]))
			accel[sp.b] -= f / m_particles[sp.b].mass;
	}

	for (int i = 0; i < n; ++i) {
		if (!m_particles[i].alive || m_particles[i].kind != KIND_BOID)
			continue;
		Vec3f sep(0, 0, 0), ali(0, 0, 0), coh(0, 0, 0);
		int neighbors = 0;
		for (int j = 0; j < n; ++j) {
			if (i == j || !m_particles[j].alive || m_particles[j].kind != KIND_BOID)
				continue;
			Vec3f d = pos[i] - pos[j];
			float dist = (float)d.length();
			if (dist < 2.4f && dist > 1e-3f) {
				if (dist < 0.75f)
					sep += d * ((0.75f - dist) / dist);
				ali += vel[j];
				coh += pos[j];
				++neighbors;
			}
		}
		Vec3f flock = sep * 6.0f;
		if (neighbors > 0) {
			flock += (ali / (float)neighbors - vel[i]) * 1.1f;
			flock += (coh / (float)neighbors - pos[i]) * 0.35f;
		}
		if (pos[i][1] < 3.2f) flock += Vec3f(0, 5.0f, 0);
		if (pos[i][1] > 9.0f) flock += Vec3f(0, -3.0f, 0);
		if (fabsf(pos[i][0]) > 10.0f) flock[0] -= pos[i][0] * 0.35f;
		if (fabsf(pos[i][2]) > 10.0f) flock[2] -= pos[i][2] * 0.35f;
		flock += Vec3f(sinf(m_particles[i].age * 1.7f + i), 0.0f, cosf(m_particles[i].age + i));
		accel[i] += flock;
	}

	for (int i = 0; i < n; ++i) {
		if (isPinned(m_particles[i]) || !m_particles[i].alive)
			accel[i] = Vec3f(0, 0, 0);
	}
}

void ParticleSystem::applyPins(std::vector<Vec3f>& pos, std::vector<Vec3f>& vel) const
{
	for (int i = 0; i < (int)m_particles.size(); ++i) {
		if (!isPinned(m_particles[i]))
			continue;
		pos[i] = pinTarget(m_particles[i]);
		vel[i] = Vec3f(0, 0, 0);
	}
}

void ParticleSystem::collide(std::vector<Vec3f>& pos, std::vector<Vec3f>& vel) const
{
	float cx = 4.4f, cz = 1.6f;
	Vec3f sphereC(cx, terrainHeight(cx, cz) + 0.75f, cz);
	float sphereR = 0.82f;
	for (int i = 0; i < (int)m_particles.size(); ++i) {
		const Particle& p = m_particles[i];
		if (!p.alive || p.kind == KIND_FIRE || p.kind == KIND_BOID || isPinned(p))
			continue;
		float ground = terrainHeight(pos[i][0], pos[i][2]) + p.radius;
		if (pos[i][1] < ground) {
			pos[i][1] = ground;
			float bounce = (p.kind == KIND_WATER) ? 0.35f : 0.08f;
			if (vel[i][1] < 0.0f)
				vel[i][1] = -vel[i][1] * bounce;
			vel[i][0] *= 0.8f;
			vel[i][2] *= 0.8f;
		}
		if (p.kind == KIND_FUR || p.kind == KIND_CABLE || p.kind == KIND_CLOTH) {
			Vec3f d = pos[i] - sphereC;
			float len = (float)d.length();
			if (len < sphereR && len > 1e-4f) {
				d /= len;
				pos[i] = sphereC + d * sphereR;
				float vn = vel[i] * d;
				if (vn < 0.0f)
					vel[i] -= d * ((1.0f + 0.15f) * vn);
			}
		}
	}

	std::vector<int> water;
	for (int i = 0; i < (int)m_particles.size(); ++i) {
		if (m_particles[i].alive && m_particles[i].kind == KIND_WATER)
			water.push_back(i);
	}
	int limit = (int)water.size();
	if (limit > 80) limit = 80;
	for (int a = 0; a < limit; ++a) {
		for (int b = a + 1; b < limit; ++b) {
			int i = water[a], j = water[b];
			Vec3f d = pos[j] - pos[i];
			float minDist = m_particles[i].radius + m_particles[j].radius;
			float dist = (float)d.length();
			if (dist < minDist && dist > 1e-4f) {
				Vec3f n = d / dist;
				float overlap = minDist - dist;
				pos[i] -= n * (overlap * 0.5f);
				pos[j] += n * (overlap * 0.5f);
				float vi = vel[i] * n;
				float vj = vel[j] * n;
				float bounce = 0.6f;
				if (vi > vj) {
					vel[i] += n * ((vj - vi) * bounce);
					vel[j] += n * ((vi - vj) * bounce);
				}
			}
		}
	}
}

void ParticleSystem::integrate(float h, bool rk4, const Vec3f& wind)
{
	int n = (int)m_particles.size();
	std::vector<Vec3f> pos(n), vel(n), accel(n);
	for (int i = 0; i < n; ++i) {
		pos[i] = m_particles[i].pos;
		vel[i] = m_particles[i].vel;
	}
	applyPins(pos, vel);

	if (!rk4) {
		computeAccel(pos, vel, accel, wind);
		for (int i = 0; i < n; ++i) {
			if (!m_particles[i].alive || isPinned(m_particles[i]))
				continue;
			vel[i] += accel[i] * h;
			pos[i] += vel[i] * h;
		}
	}
	else {
		std::vector<Vec3f> k1v(n), k1a(n), k2v(n), k2a(n), k3v(n), k3a(n), k4v(n), k4a(n);
		computeAccel(pos, vel, k1a, wind);
		k1v = vel;
		std::vector<Vec3f> p2(n), v2(n);
		for (int i = 0; i < n; ++i) {
			p2[i] = pos[i] + k1v[i] * (h * 0.5f);
			v2[i] = vel[i] + k1a[i] * (h * 0.5f);
		}
		applyPins(p2, v2);
		computeAccel(p2, v2, k2a, wind);
		k2v = v2;
		std::vector<Vec3f> p3(n), v3(n);
		for (int i = 0; i < n; ++i) {
			p3[i] = pos[i] + k2v[i] * (h * 0.5f);
			v3[i] = vel[i] + k2a[i] * (h * 0.5f);
		}
		applyPins(p3, v3);
		computeAccel(p3, v3, k3a, wind);
		k3v = v3;
		std::vector<Vec3f> p4(n), v4(n);
		for (int i = 0; i < n; ++i) {
			p4[i] = pos[i] + k3v[i] * h;
			v4[i] = vel[i] + k3a[i] * h;
		}
		applyPins(p4, v4);
		computeAccel(p4, v4, k4a, wind);
		k4v = v4;
		for (int i = 0; i < n; ++i) {
			if (!m_particles[i].alive || isPinned(m_particles[i]))
				continue;
			pos[i] += (k1v[i] + k2v[i] * 2.0f + k3v[i] * 2.0f + k4v[i]) * (h / 6.0f);
			vel[i] += (k1a[i] + k2a[i] * 2.0f + k3a[i] * 2.0f + k4a[i]) * (h / 6.0f);
		}
	}

	applyPins(pos, vel);
	collide(pos, vel);
	for (int i = 0; i < n; ++i) {
		if (!m_particles[i].alive)
			continue;
		if (m_particles[i].kind == KIND_BOID) {
			float sp = (float)vel[i].length();
			if (sp > 3.5f) vel[i] *= 3.5f / sp;
			if (sp < 0.8f && sp > 1e-4f) vel[i] *= 0.8f / sp;
		}
		else if (m_particles[i].kind == KIND_CLOTH || m_particles[i].kind == KIND_FUR
			|| m_particles[i].kind == KIND_CABLE) {
			float sp = (float)vel[i].length();
			if (sp > 4.0f)
				vel[i] *= 4.0f / sp;
		}
		m_particles[i].pos = pos[i];
		m_particles[i].vel = vel[i];
		m_particles[i].age += h;
		if ((m_particles[i].kind == KIND_FIRE || m_particles[i].kind == KIND_SNOW || m_particles[i].kind == KIND_WATER)
			&& m_particles[i].age >= m_particles[i].life)
			m_particles[i].alive = false;
	}
}

void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	Mat4f view = readModelView();
	m_invView = rigidInverse(view);
	m_viewReady = true;

	if (!simulate)
		return;
	int frame = frameIndex(t, bake_fps);
	if (m_bake.find(frame) != m_bake.end())
		return;

	float dt = 0.0f;
	if (m_hasTime)
		dt = t - m_lastTime;
	m_lastTime = t;
	m_hasTime = true;
	if (dt <= 1e-5f)
		return;
	if (dt > 0.05f)
		dt = 0.05f;

	if (!m_structuresReady && m_emit[EMIT_CLAW].valid && m_emit[EMIT_ARM].valid
		&& (int)m_emit[EMIT_ARM].pins.size() == m_clothCols
		&& (int)m_emit[EMIT_ARM].furPins.size() == m_furStrands) {
		buildStructures();
	}
	if (m_structuresReady)
		spawnEphemeral(dt);

	Vec3f wind(1.0f, 0.0f, 0.35f);
	bool rk4 = false;
	ModelerApplication* app = ModelerApplication::Instance();
	if (app) {
		float w = (float)app->GetControlValue(WIND_STRENGTH);
		wind = Vec3f(w, 0.0f, w * 0.35f);
		rk4 = app->GetControlValue(USE_RK4) >= 0.5;
	}

	int sub = (int)ceilf(dt / 0.01f);
	if (sub < 1) sub = 1;
	if (sub > 6) sub = 6;
	float h = dt / (float)sub;
	for (int s = 0; s < sub; ++s)
		integrate(h, rk4, wind);

	bakeParticles(t);
}

void ParticleSystem::bakeParticles(float t)
{
	int frame = frameIndex(t, bake_fps);
	m_bake[frame] = m_particles;
	baked = true;
	if (t < bake_start_time || bake_start_time == 0.0f)
		bake_start_time = t;
}

void ParticleSystem::drawParticles(float t)
{
	ensureParticleTextures();
	const std::vector<Particle>* src = &m_particles;
	int frame = frameIndex(t, bake_fps);
	std::map<int, std::vector<Particle> >::const_iterator it = m_bake.find(frame);
	if (it == m_bake.end() && !m_bake.empty() && !simulate) {
		it = m_bake.lower_bound(frame);
		if (it == m_bake.end()) {
			--it;
		}
		else if (it != m_bake.begin() && it->first != frame) {
			std::map<int, std::vector<Particle> >::const_iterator prev = it;
			--prev;
			if (frame - prev->first <= it->first - frame)
				it = prev;
		}
	}
	if (it != m_bake.end())
		src = &it->second;

	GLfloat mv[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, mv);
	Vec3f right(mv[0], mv[4], mv[8]);
	Vec3f up(mv[1], mv[5], mv[9]);
	safeNormalize(right);
	safeNormalize(up);

	glDisable(GL_LIGHTING);
	glLineWidth(2.0f);
	glBegin(GL_LINE_STRIP);
	glColor3f(0.8f, 0.8f, 0.85f);
	for (int i = 0; i < (int)src->size(); ++i) {
		if ((*src)[i].alive && (*src)[i].kind == KIND_CABLE)
			glVertex3f((*src)[i].pos[0], (*src)[i].pos[1], (*src)[i].pos[2]);
	}
	glEnd();

	if (m_structuresReady && m_clothCols > 1) {
		glEnable(GL_LIGHTING);
		glDisable(GL_CULL_FACE);
		GLboolean twoSide = GL_FALSE;
		glGetBooleanv(GL_LIGHT_MODEL_TWO_SIDE, &twoSide);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
		glColor3f(0.85f, 0.22f, 0.18f);
		glBegin(GL_TRIANGLES);
		for (int r = 0; r < m_clothRows - 1; ++r) {
			for (int c = 0; c < m_clothCols - 1; ++c) {
				int i00 = m_clothStart + r * m_clothCols + c;
				int i10 = i00 + 1;
				int i01 = i00 + m_clothCols;
				int i11 = i01 + 1;
				if (i11 >= (int)src->size())
					continue;
				const Particle& a = (*src)[i00];
				const Particle& b = (*src)[i10];
				const Particle& d = (*src)[i01];
				const Particle& e = (*src)[i11];
				if (!a.alive || !b.alive || !d.alive || !e.alive)
					continue;
				if ((b.pos - a.pos).length2() > 0.64f || (d.pos - a.pos).length2() > 0.64f
					|| (e.pos - b.pos).length2() > 0.64f)
					continue;
				Vec3f n = (b.pos - a.pos) ^ (d.pos - a.pos);
				safeNormalize(n);
				glNormal3f(n[0], n[1], n[2]);
				glVertex3f(a.pos[0], a.pos[1], a.pos[2]);
				glVertex3f(b.pos[0], b.pos[1], b.pos[2]);
				glVertex3f(d.pos[0], d.pos[1], d.pos[2]);
				glVertex3f(b.pos[0], b.pos[1], b.pos[2]);
				glVertex3f(e.pos[0], e.pos[1], e.pos[2]);
				glVertex3f(d.pos[0], d.pos[1], d.pos[2]);
			}
		}
		glEnd();
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, twoSide ? GL_TRUE : GL_FALSE);
		glDisable(GL_LIGHTING);
	}

	glColor3f(0.55f, 0.38f, 0.2f);
	for (int s = 0; s < m_furStrands; ++s) {
		glBegin(GL_LINE_STRIP);
		for (int k = 0; k < m_furLen; ++k) {
			int i = m_furStart + s * m_furLen + k;
			if (i >= (int)src->size() || !(*src)[i].alive || (*src)[i].kind != KIND_FUR)
				continue;
			glVertex3f((*src)[i].pos[0], (*src)[i].pos[1], (*src)[i].pos[2]);
		}
		glEnd();
	}

	glEnable(GL_LIGHTING);
	for (int i = 0; i < (int)src->size(); ++i) {
		const Particle& p = (*src)[i];
		if (!p.alive || p.kind != KIND_BOID)
			continue;
		Vec3f fwd = p.vel;
		if (fwd.length2() < 1e-4)
			fwd = Vec3f(0, 0, 1);
		safeNormalize(fwd);
		Vec3f worldUp(0, 1, 0);
		if (fabsf(fwd * worldUp) > 0.92f)
			worldUp = Vec3f(1, 0, 0);
		Vec3f side = fwd ^ worldUp;
		safeNormalize(side);
		Vec3f lift = side ^ fwd;
		safeNormalize(lift);
		float s = p.size;
		Vec3f tip = p.pos + fwd * s;
		Vec3f base = p.pos - fwd * (s * 0.45f);
		float w = s * 0.28f;
		Vec3f q1 = base + side * w;
		Vec3f q2 = base - side * w * 0.5f + lift * w;
		Vec3f q3 = base - side * w * 0.5f - lift * w;
		glColor3f(p.color[0], p.color[1], p.color[2]);
		glBegin(GL_TRIANGLES);
		Vec3f n = (q1 - tip) ^ (q2 - tip);
		safeNormalize(n);
		glNormal3f(n[0], n[1], n[2]);
		glVertex3f(tip[0], tip[1], tip[2]);
		glVertex3f(q1[0], q1[1], q1[2]);
		glVertex3f(q2[0], q2[1], q2[2]);
		n = (q2 - tip) ^ (q3 - tip);
		safeNormalize(n);
		glNormal3f(n[0], n[1], n[2]);
		glVertex3f(tip[0], tip[1], tip[2]);
		glVertex3f(q2[0], q2[1], q2[2]);
		glVertex3f(q3[0], q3[1], q3[2]);
		n = (q3 - tip) ^ (q1 - tip);
		safeNormalize(n);
		glNormal3f(n[0], n[1], n[2]);
		glVertex3f(tip[0], tip[1], tip[2]);
		glVertex3f(q3[0], q3[1], q3[2]);
		glVertex3f(q1[0], q1[1], q1[2]);
		glEnd();
	}

	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	glDepthMask(GL_FALSE);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glBindTexture(GL_TEXTURE_2D, gFireTex);
	glBegin(GL_QUADS);
	for (int i = 0; i < (int)src->size(); ++i) {
		const Particle& p = (*src)[i];
		if (!p.alive || p.kind != KIND_FIRE)
			continue;
		float fade = 1.0f - p.age / (p.life > 0.01f ? p.life : 0.01f);
		if (fade < 0.0f) fade = 0.0f;
		float sz = p.size * (1.0f + p.age * 0.8f);
		glColor4f(p.color[0], p.color[1] * fade, p.color[2] * fade, fade);
		Vec3f r = right * sz, u = up * sz;
		Vec3f p0 = p.pos - r - u, p1 = p.pos + r - u, p2 = p.pos + r + u, p3 = p.pos - r + u;
		glTexCoord2f(0, 0); glVertex3f(p0[0], p0[1], p0[2]);
		glTexCoord2f(1, 0); glVertex3f(p1[0], p1[1], p1[2]);
		glTexCoord2f(1, 1); glVertex3f(p2[0], p2[1], p2[2]);
		glTexCoord2f(0, 1); glVertex3f(p3[0], p3[1], p3[2]);
	}
	glEnd();

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBindTexture(GL_TEXTURE_2D, gWaterTex);
	glBegin(GL_QUADS);
	for (int i = 0; i < (int)src->size(); ++i) {
		const Particle& p = (*src)[i];
		if (!p.alive || p.kind != KIND_WATER)
			continue;
		float fade = 1.0f - p.age / (p.life > 0.01f ? p.life : 0.01f);
		if (fade < 0.0f) fade = 0.0f;
		glColor4f(p.color[0], p.color[1], p.color[2], 0.75f * fade);
		Vec3f r = right * p.size, u = up * p.size;
		Vec3f p0 = p.pos - r - u, p1 = p.pos + r - u, p2 = p.pos + r + u, p3 = p.pos - r + u;
		glTexCoord2f(0, 0); glVertex3f(p0[0], p0[1], p0[2]);
		glTexCoord2f(1, 0); glVertex3f(p1[0], p1[1], p1[2]);
		glTexCoord2f(1, 1); glVertex3f(p2[0], p2[1], p2[2]);
		glTexCoord2f(0, 1); glVertex3f(p3[0], p3[1], p3[2]);
	}
	glEnd();

	glBindTexture(GL_TEXTURE_2D, gSnowTex);
	glBegin(GL_QUADS);
	for (int i = 0; i < (int)src->size(); ++i) {
		const Particle& p = (*src)[i];
		if (!p.alive || p.kind != KIND_SNOW)
			continue;
		float fade = 1.0f - p.age / (p.life > 0.01f ? p.life : 0.01f);
		if (fade < 0.0f) fade = 0.0f;
		glColor4f(1, 1, 1, 0.9f * fade);
		Vec3f r = right * p.size, u = up * p.size;
		Vec3f p0 = p.pos - r - u, p1 = p.pos + r - u, p2 = p.pos + r + u, p3 = p.pos - r + u;
		glTexCoord2f(0, 0); glVertex3f(p0[0], p0[1], p0[2]);
		glTexCoord2f(1, 0); glVertex3f(p1[0], p1[1], p1[2]);
		glTexCoord2f(1, 1); glVertex3f(p2[0], p2[1], p2[2]);
		glTexCoord2f(0, 1); glVertex3f(p3[0], p3[1], p3[2]);
	}
	glEnd();

	glDepthMask(GL_TRUE);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
}
