#ifndef ANIMCONTROLS_H
#define ANIMCONTROLS_H

// Project 2 cat controls, followed by the animator curve and particle
// sliders. One enum so the graph, the cat, and the particle system
// agree on slider indices.
enum CatModelControls
{
	// Position
	XPOS, YPOS, ZPOS,

	// Individual joints
	HEAD_YAW, HEAD_PITCH,
	TAIL_BASE, TAIL_MID, TAIL_TIP,
	LEG_FL, LEG_FR, LEG_HL, LEG_HR,

	// Multi-joint / coupled actions
	STRETCH,
	SIT,
	POUNCE,

	// Detail / animation / mood
	LOD,
	MOOD,
	ANIM_SPEED,
	ANIM_MODE,

	// Individuality
	FUR_ORANGE,
	FLUFFINESS,
	EAR_SIZE,
	LEG_LENGTH,
	FACE_WIDTH,
	STRIPE_AMOUNT,
	TAIL_FLUFF,

	// L-system plant
	PLANT_DEPTH,
	PLANT_ANGLE,

	// Scene props
	TEXTURE_ON,
	FRAME_ALL,
	LIGHT_DRAMA,
	SHOW_BOWL,
	SHOW_COLLAR,
	SHOW_SHADOW,
	SHOW_YARN,
	SHOW_CUSHION,
	SHOW_MOUSE,
	BLINK,
	BREATHE,

	// Animator
	PARTICLE_COUNT,
	WIND_STRENGTH,
	CATMULL_TENSION,
	BEZIER_FLATNESS,
	ADAPTIVE_BEZIER,
	BEZIER_CONTINUITY,
	BSPLINE_DEGREE,
	LANE_RIESENFELD,
	C2_CLAMPED,
	C2_DERIV0,
	C2_DERIV1,
	USE_RK4,

	NUMCONTROLS
};

// Old name used by the curve and particle code.
typedef CatModelControls RobotArmControls;

#endif
