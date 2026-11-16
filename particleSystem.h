/***********************
 * ParticleSystem class
 ***********************/

#ifndef __PARTICLE_SYSTEM_H__
#define __PARTICLE_SYSTEM_H__

#include "vec.h"
#include "mat.h"
#include "particle.h"
#include <vector>
#include <map>

class ParticleSystem {
public:
	ParticleSystem();
	virtual ~ParticleSystem();

	virtual void drawParticles(float t);
	virtual void bakeParticles(float t);
	virtual void computeForcesAndUpdateParticles(float t);
	virtual void resetSimulation(float t);
	virtual void startSimulation(float t);
	virtual void stopSimulation(float t);
	virtual void clearBaked();

	// Called from the model hierarchy, after the OpenGL matrix
	// has been moved to the emitting part. Positions are stored in world space.
	void setEmitter(int emitterId, float dirX, float dirY, float dirZ);
	void setArmPins(float armHeight);

	float getBakeStartTime() { return bake_start_time; }
	float getBakeEndTime() { return bake_end_time; }
	float getBakeFps() { return bake_fps; }
	bool isSimulate() { return simulate; }
	bool isDirty() { return dirty; }
	void setDirty(bool d) { dirty = d; }

protected:
	float bake_fps;
	float bake_start_time;
	float bake_end_time;
	bool baked;
	bool simulate;
	bool dirty;

	struct Emitter {
		Vec3f pos;
		Vec3f dir;
		Vec3f vel;
		float lastT;
		bool valid;
		std::vector<Vec3f> pins;
		std::vector<Vec3f> furPins;
		std::vector<Vec3f> furDirs;
		Emitter() : pos(0, 0, 0), dir(0, 1, 0), vel(0, 0, 0), lastT(-1.0f), valid(false) {}
	};

	struct Spring {
		int a, b;
		float rest;
		float ks;
		float kd;
	};

	Emitter m_emit[EMIT_COUNT];
	std::vector<Particle> m_particles;
	std::vector<Spring> m_springs;
	std::map<int, std::vector<Particle> > m_bake;
	bool m_viewReady;
	Mat4f m_invView;
	bool m_structuresReady;
	bool m_hasTime;
	float m_lastTime;
	float m_fireAcc, m_snowAcc, m_waterAcc;
	int m_clothStart, m_clothCols, m_clothRows;
	int m_cableStart, m_cableCount;
	int m_furStart, m_furStrands, m_furLen;
	int m_boidStart, m_boidCount;

	void addSpring(int a, int b, float ks, float kd);
	void buildStructures();
	bool isPinned(const Particle& p) const;
	Vec3f pinTarget(const Particle& p) const;
	int acquire(int kind, int maxCount);
	void spawnEphemeral(float dt);
	void computeAccel(const std::vector<Vec3f>& pos, const std::vector<Vec3f>& vel,
		std::vector<Vec3f>& accel, const Vec3f& wind) const;
	void applyPins(std::vector<Vec3f>& pos, std::vector<Vec3f>& vel) const;
	void collide(std::vector<Vec3f>& pos, std::vector<Vec3f>& vel) const;
	void integrate(float h, bool rk4, const Vec3f& wind);
};

#endif
