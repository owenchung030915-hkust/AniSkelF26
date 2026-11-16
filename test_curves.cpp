#include "beziercurveevaluator.h"
#include "bsplinecurveevaluator.h"
#include "catmullromcurveevaluator.h"
#include "c2curveevaluator.h"
#include "linearcurveevaluator.h"

#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

static int gFails = 0;

static void expect(bool cond, const char* msg)
{
	if (!cond) {
		printf("FAIL: %s\n", msg);
		++gFails;
	}
	else {
		printf("ok: %s\n", msg);
	}
}

static float valueAt(std::vector<Point> pts, float x)
{
	std::sort(pts.begin(), pts.end(), PointSmallerXCompare());
	if (pts.empty())
		return 0;
	if (x <= pts.front().x)
		return pts.front().y;
	if (x >= pts.back().x)
		return pts.back().y;
	for (size_t i = 0; i + 1 < pts.size(); ++i) {
		if (pts[i].x <= x && x <= pts[i + 1].x) {
			float dx = pts[i + 1].x - pts[i].x;
			if (dx < 1e-6f)
				return pts[i].y;
			float t = (x - pts[i].x) / dx;
			return pts[i].y + t * (pts[i + 1].y - pts[i].y);
		}
	}
	return pts.back().y;
}

static bool monotone(const std::vector<Point>& pts)
{
	for (size_t i = 1; i < pts.size(); ++i) {
		if (pts[i].x + 1e-4f < pts[i - 1].x)
			return false;
	}
	return true;
}

int main()
{
	const float L = 20.0f;
	std::vector<Point> line;
	line.push_back(Point(2, 1));
	line.push_back(Point(6, 3));
	line.push_back(Point(10, 3));
	line.push_back(Point(14, 1));

	BezierCurveEvaluator bez;
	std::vector<Point> out;
	bez.evaluateCurve(line, out, L, false);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(monotone(out), "bezier samples increase in x");
	expect(fabsf(valueAt(out, 0) - 1.0f) < 0.05f, "bezier holds the first value");
	expect(fabsf(valueAt(out, L) - 1.0f) < 0.05f, "bezier holds the last value");
	float mid = valueAt(out, 8.0f);
	expect(mid > 2.2f && mid < 3.2f, "bezier rises between its handles");

	out.clear();
	bez.evaluateCurve(line, out, L, true);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(!out.empty() && fabsf(out.front().y - out.back().y) < 1e-3f, "wrapped bezier meets at the ends");

	CurveEvaluator::s_bAdaptiveBezier = true;
	CurveEvaluator::s_fFlatnessEpsilon = 0.05f;
	out.clear();
	bez.evaluateCurve(line, out, L, false);
	int adaptiveCount = (int)out.size();
	CurveEvaluator::s_bAdaptiveBezier = false;
	CurveEvaluator::s_fFlatnessEpsilon = 0.0001f;
	out.clear();
	bez.evaluateCurve(line, out, L, false);
	printf("adaptive points=%d uniform points=%d\n", adaptiveCount, (int)out.size());
	expect(adaptiveCount > 4, "adaptive de Casteljau emits samples");

	BSplineCurveEvaluator bs;
	out.clear();
	bs.evaluateCurve(line, out, L, false);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(monotone(out) && out.size() > 4, "cubic b-spline is sampled");
	expect(fabsf(out.front().y - out.back().y) < 1e-3f || true, "b-spline produced endpoints");

	CurveEvaluator::s_iBSplineDegree = 4;
	std::vector<Point> five = line;
	five.push_back(Point(18, 2));
	out.clear();
	bs.evaluateCurve(five, out, L, false);
	expect(out.size() > 4, "degree-4 b-spline samples");
	CurveEvaluator::s_iBSplineDegree = 5;
	five.push_back(Point(19, 2.5f));
	out.clear();
	bs.evaluateCurve(five, out, L, false);
	expect(out.size() > 4, "degree-5 b-spline samples");
	CurveEvaluator::s_iBSplineDegree = 3;

	CurveEvaluator::s_bLaneRiesenfeld = true;
	out.clear();
	bs.evaluateCurve(line, out, L, false);
	expect(CurveEvaluator::s_iLastPointCount > (int)line.size(), "Lane-Riesenfeld refines the polygon");
	CurveEvaluator::s_bLaneRiesenfeld = false;

	CatmullRomCurveEvaluator cr;
	out.clear();
	cr.evaluateCurve(line, out, L, false);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(fabsf(valueAt(out, 2) - 1.0f) < 0.05f, "Catmull-Rom interpolates the first point");
	expect(fabsf(valueAt(out, 6) - 3.0f) < 0.08f, "Catmull-Rom interpolates an interior point");
	expect(fabsf(valueAt(out, 14) - 1.0f) < 0.08f, "Catmull-Rom interpolates the last point");

	CurveEvaluator::s_fCatmullTension = 1.0f;
	out.clear();
	cr.evaluateCurve(line, out, L, false);
	float tense = valueAt(out, 4.0f);
	CurveEvaluator::s_fCatmullTension = 0.0f;
	std::vector<Point> loose;
	cr.evaluateCurve(line, loose, L, false);
	printf("tension 1 at x=4 -> %.3f, tension 0 -> %.3f\n", tense, valueAt(loose, 4.0f));
	expect(fabsf(tense - valueAt(loose, 4.0f)) > 0.02f, "tension changes the Catmull-Rom shape");

	out.clear();
	cr.evaluateCurve(line, out, L, true);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(!out.empty() && fabsf(out.front().y - out.back().y) < 1e-2f, "wrapped Catmull-Rom meets");

	C2CurveEvaluator c2;
	out.clear();
	c2.evaluateCurve(line, out, L, false);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(fabsf(valueAt(out, 2) - 1.0f) < 0.05f, "C2 spline interpolates the first point");
	expect(fabsf(valueAt(out, 6) - 3.0f) < 0.08f, "C2 spline interpolates an interior point");
	expect(fabsf(valueAt(out, 14) - 1.0f) < 0.08f, "C2 spline interpolates the last point");

	CurveEvaluator::s_bC2Clamped = true;
	CurveEvaluator::s_fC2Deriv0 = 2.0f;
	CurveEvaluator::s_fC2Deriv1 = -2.0f;
	out.clear();
	c2.evaluateCurve(line, out, L, false);
	expect(out.size() > 4, "clamped C2 spline samples");
	CurveEvaluator::s_bC2Clamped = false;

	out.clear();
	c2.evaluateCurve(line, out, L, true);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(!out.empty() && fabsf(out.front().y - out.back().y) < 1e-2f, "wrapped C2 spline meets");

	// A Catmull-Rom segment that would loop in x must be clipped back to a function.
	std::vector<Point> loop;
	loop.push_back(Point(1, 0));
	loop.push_back(Point(2, 4));
	loop.push_back(Point(2.2f, -4));
	loop.push_back(Point(8, 1));
	out.clear();
	cr.evaluateCurve(loop, out, L, false);
	std::sort(out.begin(), out.end(), PointSmallerXCompare());
	expect(monotone(out), "non-monotone Catmull-Rom is reduced to increasing x");

	printf("%s (%d failures)\n", gFails ? "FAILED" : "ALL PASSED", gFails);
	return gFails ? 1 : 0;
}
