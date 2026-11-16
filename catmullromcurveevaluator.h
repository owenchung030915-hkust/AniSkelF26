#ifndef INCLUDED_CATMULLROM_CURVE_EVALUATOR_H
#define INCLUDED_CATMULLROM_CURVE_EVALUATOR_H

#include "curveevaluator.h"

class CatmullRomCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts,
		std::vector<Point>& ptvEvaluatedCurvePts,
		const float& fAniLength,
		const bool& bWrap) const;
};

#endif
