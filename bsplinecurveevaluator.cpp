#include "bsplinecurveevaluator.h"
#include "linearcurveevaluator.h"
#include "curvesample.h"
#include <cstdio>
#include <map>

namespace {

void sampleCubic(const Point& p0, const Point& p1, const Point& p2, const Point& p3,
	int steps, std::vector<Point>& out)
{
	if (steps < 1) steps = 1;
	for (int i = 0; i <= steps; ++i) {
		float t = i / (float)steps;
		float u = 1.0f - t;
		float b0 = (u * u * u) / 6.0f;
		float b1 = (3.0f * t * t * t - 6.0f * t * t + 4.0f) / 6.0f;
		float b2 = (-3.0f * t * t * t + 3.0f * t * t + 3.0f * t + 1.0f) / 6.0f;
		float b3 = (t * t * t) / 6.0f;
		addMonotonic(out, Point(
			b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
			b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y));
	}
}

float coxDeBoor(int i, int p, float u, const std::vector<float>& U)
{
	int m = (int)U.size() - 1;
	if (p == 0) {
		if (u >= U[i] && (u < U[i + 1] || (i + 1 == m && u <= U[i + 1])))
			return 1.0f;
		return 0.0f;
	}
	float left = 0.0f, right = 0.0f;
	float d1 = U[i + p] - U[i];
	float d2 = U[i + p + 1] - U[i + 1];
	if (fabsf(d1) > 1e-8f)
		left = ((u - U[i]) / d1) * coxDeBoor(i, p - 1, u, U);
	if (fabsf(d2) > 1e-8f)
		right = ((U[i + p + 1] - u) / d2) * coxDeBoor(i + 1, p - 1, u, U);
	return left + right;
}

void sampleDegree(const std::vector<Point>& ctrl, int start, int degree, int steps,
	std::vector<Point>& out)
{
	int m = degree + 1;
	std::vector<float> U(m + degree + 1);
	for (int i = 0; i < (int)U.size(); ++i)
		U[i] = (float)i;
	if (steps < 1) steps = 1;
	for (int s = 0; s <= steps; ++s) {
		float t = s / (float)steps;
		float u = (float)degree + t;
		if (s == steps)
			u = (float)degree + 1.0f;
		float x = 0.0f, y = 0.0f, w = 0.0f;
		for (int i = 0; i < m; ++i) {
			float b = coxDeBoor(i, degree, u, U);
			w += b;
			x += b * ctrl[start + i].x;
			y += b * ctrl[start + i].y;
		}
		if (w > 1e-6f) {
			x /= w;
			y /= w;
		}
		addMonotonic(out, Point(x, y));
	}
}

void laneRiesenfeld(std::vector<Point> pts, int degree, int levels, std::vector<Point>& out)
{
	if (degree < 2) degree = 2;
	for (int level = 0; level < levels; ++level) {
		std::vector<Point> doubled;
		doubled.reserve(pts.size() * 2);
		for (size_t i = 0; i < pts.size(); ++i) {
			doubled.push_back(pts[i]);
			doubled.push_back(pts[i]);
		}
		for (int s = 0; s < degree - 1; ++s) {
			if (doubled.size() < 2)
				break;
			std::vector<Point> smoothed(doubled.size());
			smoothed[0] = doubled[0];
			for (size_t i = 1; i < doubled.size(); ++i)
				smoothed[i] = lerpPoint(doubled[i - 1], doubled[i], 0.5f);
			doubled.swap(smoothed);
		}
		pts.swap(doubled);
	}
	out.clear();
	for (size_t i = 0; i < pts.size(); ++i)
		addMonotonic(out, pts[i]);
}

void noteSubdivision(int generated)
{
	static int last = -1;
	if (generated == last)
		return;
	last = generated;
	printf("Lane-Riesenfeld subdivision produced %d points\n", generated);
	CurveEvaluator::s_iLastPointCount = generated;
}

}

void BSplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	ptvEvaluatedCurvePts.clear();
	int count = (int)ptvCtrlPts.size();
	if (count <= 0)
		return;

	int degree = s_iBSplineDegree;
	if (degree < 3) degree = 3;
	if (degree > 5) degree = 5;
	if (s_bLaneRiesenfeld)
		degree = 3;

	if (count < degree + 1) {
		LinearCurveEvaluator linear;
		linear.evaluateCurve(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
		return;
	}

	std::vector<Point> ctrl = ptvCtrlPts;
	if (bWrap) {
		for (int i = 0; i < degree && i < count; ++i)
			ctrl.push_back(Point(ptvCtrlPts[i].x + fAniLength, ptvCtrlPts[i].y));
	}

	std::vector<Point> inside, wrapped;
	if (s_bLaneRiesenfeld) {
		std::vector<Point> sub;
		laneRiesenfeld(ctrl, 3, 4, sub);
		splitSamples(sub, fAniLength, inside, wrapped);
		noteSubdivision((int)sub.size());
	}
	else {
		int n = (int)ctrl.size();
		int segments = n - degree;
		for (int i = 0; i < segments; ++i) {
			std::vector<Point> seg;
			if (degree == 3)
				sampleCubic(ctrl[i], ctrl[i + 1], ctrl[i + 2], ctrl[i + 3], s_iSegCount, seg);
			else
				sampleDegree(ctrl, i, degree, s_iSegCount, seg);
			splitSamples(seg, fAniLength, inside, wrapped);
		}
		CurveEvaluator::s_iLastPointCount = (int)inside.size();
	}

	if (bWrap)
		ptvEvaluatedCurvePts = mergeWrapped(inside, wrapped, fAniLength);
	else {
		ptvEvaluatedCurvePts = inside;
		float y0 = inside.empty() ? ptvCtrlPts.front().y : inside.front().y;
		float y1 = inside.empty() ? ptvCtrlPts.back().y : inside.back().y;
		extendHorizontal(ptvEvaluatedCurvePts, fAniLength, y0, y1);
	}
}
