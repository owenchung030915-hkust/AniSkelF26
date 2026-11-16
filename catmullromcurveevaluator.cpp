#include "catmullromcurveevaluator.h"
#include "linearcurveevaluator.h"
#include "curvesample.h"

namespace {

void sampleSegment(const Point& p0, const Point& p1, const Point& p2, const Point& p3,
	float tension, int steps, std::vector<Point>& out)
{
	float s = (1.0f - tension) * 0.5f;
	float m1x = s * (p2.x - p0.x);
	float m1y = s * (p2.y - p0.y);
	float m2x = s * (p3.x - p1.x);
	float m2y = s * (p3.y - p1.y);
	if (steps < 1) steps = 1;
	for (int i = 0; i <= steps; ++i) {
		float t = i / (float)steps;
		float t2 = t * t;
		float t3 = t2 * t;
		float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
		float h10 = t3 - 2.0f * t2 + t;
		float h01 = -2.0f * t3 + 3.0f * t2;
		float h11 = t3 - t2;
		addMonotonic(out, Point(
			h00 * p1.x + h10 * m1x + h01 * p2.x + h11 * m2x,
			h00 * p1.y + h10 * m1y + h01 * p2.y + h11 * m2y));
	}
}

}

void CatmullRomCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	ptvEvaluatedCurvePts.clear();
	int count = (int)ptvCtrlPts.size();
	if (count <= 0)
		return;
	if (count < 2) {
		LinearCurveEvaluator linear;
		linear.evaluateCurve(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
		return;
	}

	float tension = s_fCatmullTension;
	if (tension < -1.0f) tension = -1.0f;
	if (tension > 1.0f) tension = 1.0f;

	std::vector<Point> inside, wrapped;
	for (int i = 0; i < count - 1; ++i) {
		Point pm = (i == 0)
			? (bWrap ? Point(ptvCtrlPts[count - 1].x - fAniLength, ptvCtrlPts[count - 1].y) : ptvCtrlPts[0])
			: ptvCtrlPts[i - 1];
		Point p0 = ptvCtrlPts[i];
		Point p1 = ptvCtrlPts[i + 1];
		Point pp = (i + 2 >= count)
			? (bWrap ? Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y) : ptvCtrlPts[count - 1])
			: ptvCtrlPts[i + 2];
		std::vector<Point> seg;
		sampleSegment(pm, p0, p1, pp, tension, s_iSegCount, seg);
		splitSamples(seg, fAniLength, inside, wrapped);
	}

	if (bWrap && count >= 2) {
		Point pm = ptvCtrlPts[count - 2];
		Point p0 = ptvCtrlPts[count - 1];
		Point p1(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
		Point pp = (count >= 2)
			? Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y)
			: p1;
		std::vector<Point> seg;
		sampleSegment(pm, p0, p1, pp, tension, s_iSegCount, seg);
		splitSamples(seg, fAniLength, inside, wrapped);
		ptvEvaluatedCurvePts = mergeWrapped(inside, wrapped, fAniLength);
	}
	else {
		ptvEvaluatedCurvePts = inside;
		extendHorizontal(ptvEvaluatedCurvePts, fAniLength, ptvCtrlPts.front().y, ptvCtrlPts.back().y);
	}
}
