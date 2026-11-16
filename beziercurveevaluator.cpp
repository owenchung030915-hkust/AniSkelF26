#include "beziercurveevaluator.h"
#include "linearcurveevaluator.h"
#include "curvesample.h"
#include <cstdio>
#include <map>

namespace {

const int kMaxDepth = 10;

void sampleUniform(const Point& p0, const Point& p1, const Point& p2, const Point& p3,
	int steps, std::vector<Point>& out)
{
	if (steps < 1) steps = 1;
	for (int i = 0; i <= steps; ++i) {
		float t = i / (float)steps;
		float u = 1.0f - t;
		float b0 = u * u * u;
		float b1 = 3.0f * u * u * t;
		float b2 = 3.0f * u * t * t;
		float b3 = t * t * t;
		addMonotonic(out, Point(
			b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
			b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y));
	}
}

bool isFlat(const Point& p0, const Point& p1, const Point& p2, const Point& p3, float eps)
{
	return distToSegment(p1, p0, p3) <= eps && distToSegment(p2, p0, p3) <= eps;
}

void sampleAdaptive(const Point& p0, const Point& p1, const Point& p2, const Point& p3,
	float eps, int depth, std::vector<Point>& out)
{
	if (depth >= kMaxDepth || isFlat(p0, p1, p2, p3, eps)) {
		addMonotonic(out, p3);
		return;
	}
	Point p01 = lerpPoint(p0, p1, 0.5f);
	Point p12 = lerpPoint(p1, p2, 0.5f);
	Point p23 = lerpPoint(p2, p3, 0.5f);
	Point p012 = lerpPoint(p01, p12, 0.5f);
	Point p123 = lerpPoint(p12, p23, 0.5f);
	Point mid = lerpPoint(p012, p123, 0.5f);
	sampleAdaptive(p0, p01, p012, mid, eps, depth + 1, out);
	sampleAdaptive(mid, p123, p23, p3, eps, depth + 1, out);
}

void notePointCount(int controlCount, float aniLength, int generated, bool adaptive, float eps)
{
	CurveEvaluator::s_iLastPointCount = generated;
	if (!adaptive)
		return;
	int key = controlCount * 131 + (int)(aniLength * 10.0f);
	static std::map<int, int> seen;
	std::map<int, int>::iterator it = seen.find(key);
	if (it != seen.end() && it->second == generated)
		return;
	seen[key] = generated;
	printf("Adaptive de Casteljau: %d points (controls=%d, flatness=%g)\n",
		generated, controlCount, eps);
}

}

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	ptvEvaluatedCurvePts.clear();
	int count = (int)ptvCtrlPts.size();
	if (count <= 0)
		return;

	if (count < 4) {
		LinearCurveEvaluator linear;
		linear.evaluateCurve(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
		return;
	}

	std::vector<Point> ctrl = ptvCtrlPts;
	if (bWrap) {
		for (int i = 0; i < 3 && i < count; ++i)
			ctrl.push_back(Point(ptvCtrlPts[i].x + fAniLength, ptvCtrlPts[i].y));
	}

	// 0 = C0 (shared endpoints, step 3), 1 = C1 (mirrored handles), 2 = free segments
	int continuity = s_iBezierContinuity;
	int step = (continuity == 2) ? 4 : 3;
	bool adaptive = s_bAdaptiveBezier;
	float eps = s_fFlatnessEpsilon;
	if (eps < 1e-4f) eps = 1e-4f;

	std::vector<Point> inside, wrapped;
	int n = (int)ctrl.size();
	int i = 0;
	for (; i + 3 < n; i += step) {
		Point p0 = ctrl[i];
		Point p1 = ctrl[i + 1];
		Point p2 = ctrl[i + 2];
		Point p3 = ctrl[i + 3];
		if (continuity == 1 && i > 0) {
			Point prev = ctrl[i - 1];
			p1 = Point(2.0f * p0.x - prev.x, 2.0f * p0.y - prev.y);
		}
		std::vector<Point> seg;
		if (adaptive) {
			addMonotonic(seg, p0);
			sampleAdaptive(p0, p1, p2, p3, eps, 0, seg);
		}
		else {
			sampleUniform(p0, p1, p2, p3, s_iSegCount, seg);
		}
		splitSamples(seg, fAniLength, inside, wrapped);
	}
	for (; i < count; ++i)
		splitSamples(std::vector<Point>(1, ctrl[i]), fAniLength, inside, wrapped);

	if (bWrap)
		ptvEvaluatedCurvePts = mergeWrapped(inside, wrapped, fAniLength);
	else {
		ptvEvaluatedCurvePts = inside;
		extendHorizontal(ptvEvaluatedCurvePts, fAniLength, ptvCtrlPts.front().y, ptvCtrlPts.back().y);
	}

	notePointCount(count, fAniLength, (int)ptvEvaluatedCurvePts.size(), adaptive, eps);
}
