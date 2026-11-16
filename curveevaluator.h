#ifndef INCLUDED_CURVE_EVALUATOR_H
#define INCLUDED_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)

#include "Curve.h"

//using namespace std;

class CurveEvaluator
{
public:
	virtual ~CurveEvaluator(void);
	virtual void evaluateCurve(const std::vector<Point>& control_points, 
							   std::vector<Point>& evaluated_curve_points, 
							   const float& animation_length, 
							   const bool& wrap_control_points) const = 0;
	static float s_fFlatnessEpsilon;
	static int s_iSegCount;

	// Shared by the graph and the model sliders. Curves re-sample themselves
	// when s_iOptionGeneration changes, so these can be edited at runtime.
	static bool s_bAdaptiveBezier;
	static int s_iBezierContinuity;   // 0 C0, 1 C1, 2 independent segments
	static float s_fCatmullTension;   // 0 is standard Catmull-Rom
	static int s_iBSplineDegree;      // 3, 4 or 5 (4 and 5 are C3 / C4)
	static bool s_bLaneRiesenfeld;
	static bool s_bC2Clamped;
	static float s_fC2Deriv0;
	static float s_fC2Deriv1;
	static int s_iLastPointCount;
	static int s_iOptionGeneration;
};

void syncCurveOptions();


#endif