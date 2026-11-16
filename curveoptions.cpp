#include "curveevaluator.h"
#include "animcontrols.h"
#include "modelerapp.h"
#include <cmath>
#include <cstdio>

namespace {

bool different(float a, float b)
{
	return fabsf(a - b) > 1e-4f;
}

}

void syncCurveOptions()
{
	static bool busy = false;
	if (busy)
		return;
	ModelerApplication* app = ModelerApplication::Instance();
	if (!app)
		return;

	busy = true;
	bool changed = false;

	float tension = (float)app->GetControlValue(CATMULL_TENSION);
	float flatness = (float)app->GetControlValue(BEZIER_FLATNESS);
	bool adaptive = app->GetControlValue(ADAPTIVE_BEZIER) >= 0.5;
	int continuity = (int)(app->GetControlValue(BEZIER_CONTINUITY) + 0.5f);
	int degree = (int)(app->GetControlValue(BSPLINE_DEGREE) + 0.5f);
	bool lane = app->GetControlValue(LANE_RIESENFELD) >= 0.5;
	bool clamped = app->GetControlValue(C2_CLAMPED) >= 0.5;
	float d0 = (float)app->GetControlValue(C2_DERIV0);
	float d1 = (float)app->GetControlValue(C2_DERIV1);

	if (continuity < 0) continuity = 0;
	if (continuity > 2) continuity = 2;
	if (degree < 3) degree = 3;
	if (degree > 5) degree = 5;
	if (flatness < 0.01f) flatness = 0.01f;

	if (different(tension, CurveEvaluator::s_fCatmullTension) ||
		different(flatness, CurveEvaluator::s_fFlatnessEpsilon) ||
		adaptive != CurveEvaluator::s_bAdaptiveBezier ||
		continuity != CurveEvaluator::s_iBezierContinuity ||
		degree != CurveEvaluator::s_iBSplineDegree ||
		lane != CurveEvaluator::s_bLaneRiesenfeld ||
		clamped != CurveEvaluator::s_bC2Clamped ||
		different(d0, CurveEvaluator::s_fC2Deriv0) ||
		different(d1, CurveEvaluator::s_fC2Deriv1)) {
		changed = true;
	}

	CurveEvaluator::s_fCatmullTension = tension;
	CurveEvaluator::s_fFlatnessEpsilon = flatness;
	CurveEvaluator::s_bAdaptiveBezier = adaptive;
	CurveEvaluator::s_iBezierContinuity = continuity;
	CurveEvaluator::s_iBSplineDegree = degree;
	CurveEvaluator::s_bLaneRiesenfeld = lane;
	CurveEvaluator::s_bC2Clamped = clamped;
	CurveEvaluator::s_fC2Deriv0 = d0;
	CurveEvaluator::s_fC2Deriv1 = d1;

	if (changed) {
		CurveEvaluator::s_iOptionGeneration++;
		const char* contName = "C0";
		if (continuity == 1) contName = "C1";
		if (continuity == 2) contName = "free";
		printf("Curve options: tension=%.2f flatness=%.3f adaptive=%s bezier=%s degree=%d lane=%s c2=%s deriv=(%.2f, %.2f) pts=%d\n",
			tension, flatness, adaptive ? "on" : "off", contName, degree,
			lane ? "on" : "off", clamped ? "clamped" : "natural",
			d0, d1, CurveEvaluator::s_iLastPointCount);
	}
	busy = false;
}
