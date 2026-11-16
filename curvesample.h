#ifndef CURVESAMPLE_H
#define CURVESAMPLE_H

#include "point.h"
#include <vector>
#include <cmath>
#include <algorithm>

// Helpers shared by the spline evaluators. Evaluated points are kept
// strictly increasing in x so the graph can treat the result as a function.

inline void addMonotonic(std::vector<Point>& pts, const Point& p)
{
	if (pts.empty() || p.x > pts.back().x + 1e-5f)
		pts.push_back(p);
}

inline Point lerpPoint(const Point& a, const Point& b, float t)
{
	return Point(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
}

inline float distToSegment(const Point& p, const Point& a, const Point& b)
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	float len2 = dx * dx + dy * dy;
	if (len2 < 1e-12f)
		return p.distance(a);
	float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
	if (t < 0.0f) t = 0.0f;
	if (t > 1.0f) t = 1.0f;
	Point q(a.x + t * dx, a.y + t * dy);
	return p.distance(q);
}

inline void splitSamples(const std::vector<Point>& samples, float aniLength,
	std::vector<Point>& inside, std::vector<Point>& wrapped)
{
	for (size_t i = 0; i < samples.size(); ++i) {
		Point p = samples[i];
		if (p.x >= -1e-4f && p.x <= aniLength + 1e-4f) {
			if (p.x < 0.0f) p.x = 0.0f;
			if (p.x > aniLength) p.x = aniLength;
			addMonotonic(inside, p);
		}
		else if (p.x > aniLength) {
			addMonotonic(wrapped, Point(p.x - aniLength, p.y));
		}
		else {
			addMonotonic(wrapped, Point(p.x + aniLength, p.y));
		}
	}
}

inline std::vector<Point> mergeWrapped(std::vector<Point> inside,
	std::vector<Point> wrapped, float aniLength)
{
	std::sort(wrapped.begin(), wrapped.end(), PointSmallerXCompare());
	std::vector<Point> uniqueWrapped;
	for (size_t i = 0; i < wrapped.size(); ++i)
		addMonotonic(uniqueWrapped, wrapped[i]);

	float minX = inside.empty() ? aniLength : inside.front().x;
	float maxX = inside.empty() ? 0.0f : inside.back().x;

	std::vector<Point> out;
	for (size_t i = 0; i < uniqueWrapped.size(); ++i) {
		if (uniqueWrapped[i].x < minX - 1e-4f)
			addMonotonic(out, uniqueWrapped[i]);
	}
	for (size_t i = 0; i < inside.size(); ++i)
		addMonotonic(out, inside[i]);
	for (size_t i = 0; i < uniqueWrapped.size(); ++i) {
		if (uniqueWrapped[i].x > maxX + 1e-4f)
			addMonotonic(out, uniqueWrapped[i]);
	}

	if (!out.empty()) {
		float yJoin = 0.5f * (out.front().y + out.back().y);
		if (out.front().x > 1e-3f)
			out.insert(out.begin(), Point(0.0f, yJoin));
		else {
			out.front().x = 0.0f;
			out.front().y = yJoin;
		}
		if (out.back().x < aniLength - 1e-3f)
			out.push_back(Point(aniLength, yJoin));
		else {
			out.back().x = aniLength;
			out.back().y = yJoin;
		}
	}
	return out;
}

inline void extendHorizontal(std::vector<Point>& pts, float aniLength, float y0, float y1)
{
	if (pts.empty() || pts.front().x > 1e-4f)
		pts.insert(pts.begin(), Point(0.0f, y0));
	else
		pts.front().x = 0.0f;

	if (pts.back().x < aniLength - 1e-4f)
		pts.push_back(Point(aniLength, y1));
	else {
		pts.back().x = aniLength;
		pts.back().y = y1;
	}
}

#endif
