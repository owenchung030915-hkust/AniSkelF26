#include "c2curveevaluator.h"
#include "linearcurveevaluator.h"
#include "curvesample.h"

namespace {

bool solveLinear(std::vector<std::vector<float> > A, std::vector<float> b, std::vector<float>& x)
{
	int n = (int)b.size();
	for (int k = 0; k < n; ++k) {
		int piv = k;
		for (int i = k + 1; i < n; ++i) {
			if (fabsf(A[i][k]) > fabsf(A[piv][k]))
				piv = i;
		}
		if (fabsf(A[piv][k]) < 1e-8f)
			return false;
		std::swap(A[k], A[piv]);
		std::swap(b[k], b[piv]);
		float diag = A[k][k];
		for (int j = k; j < n; ++j)
			A[k][j] /= diag;
		b[k] /= diag;
		for (int i = 0; i < n; ++i) {
			if (i == k) continue;
			float f = A[i][k];
			if (f == 0.0f) continue;
			for (int j = k; j < n; ++j)
				A[i][j] -= f * A[k][j];
			b[i] -= f * b[k];
		}
	}
	x = b;
	return true;
}

void sampleSpan(float x0, float x1, float y0, float y1, float z0, float z1,
	int steps, float aniLength, std::vector<Point>& inside, std::vector<Point>& wrapped)
{
	float h = x1 - x0;
	if (h < 1e-5f)
		return;
	if (steps < 1) steps = 1;
	std::vector<Point> seg;
	for (int s = 0; s <= steps; ++s) {
		float u = h * (s / (float)steps);
		float A = (z1 - z0) / (6.0f * h);
		float B = z0 * 0.5f;
		float C = (y1 - y0) / h - h * (2.0f * z0 + z1) / 6.0f;
		float y = ((A * u + B) * u + C) * u + y0;
		addMonotonic(seg, Point(x0 + u, y));
	}
	splitSamples(seg, aniLength, inside, wrapped);
}

}

void C2CurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	ptvEvaluatedCurvePts.clear();
	int n = (int)ptvCtrlPts.size();
	if (n <= 0)
		return;
	if (n < 3) {
		LinearCurveEvaluator linear;
		linear.evaluateCurve(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
		return;
	}

	std::vector<float> x(n), y(n);
	for (int i = 0; i < n; ++i) {
		x[i] = ptvCtrlPts[i].x;
		y[i] = ptvCtrlPts[i].y;
	}

	std::vector<float> h(n, 0.0f);
	std::vector<float> delta(n, 0.0f);
	int intervals = bWrap ? n : n - 1;
	for (int i = 0; i < intervals; ++i) {
		int j = (i + 1) % n;
		float x1 = x[j];
		if (bWrap && j == 0)
			x1 += fAniLength;
		h[i] = x1 - x[i];
		if (h[i] < 1e-4f)
			h[i] = 1e-4f;
		delta[i] = (y[j] - y[i]) / h[i];
	}

	std::vector<float> z(n, 0.0f);
	bool solved = false;

	if (bWrap) {
		std::vector<std::vector<float> > A(n, std::vector<float>(n, 0.0f));
		std::vector<float> rhs(n, 0.0f);
		for (int i = 0; i < n; ++i) {
			int prev = (i - 1 + n) % n;
			int next = (i + 1) % n;
			A[i][prev] += h[prev];
			A[i][i] += 2.0f * (h[prev] + h[i]);
			A[i][next] += h[i];
			rhs[i] = 6.0f * (delta[i] - delta[prev]);
		}
		solved = solveLinear(A, rhs, z);
	}
	else if (s_bC2Clamped) {
		std::vector<std::vector<float> > A(n, std::vector<float>(n, 0.0f));
		std::vector<float> rhs(n, 0.0f);
		A[0][0] = 2.0f * h[0];
		A[0][1] = h[0];
		rhs[0] = 6.0f * (delta[0] - s_fC2Deriv0);
		for (int i = 1; i < n - 1; ++i) {
			A[i][i - 1] = h[i - 1];
			A[i][i] = 2.0f * (h[i - 1] + h[i]);
			A[i][i + 1] = h[i];
			rhs[i] = 6.0f * (delta[i] - delta[i - 1]);
		}
		A[n - 1][n - 2] = h[n - 2];
		A[n - 1][n - 1] = 2.0f * h[n - 2];
		rhs[n - 1] = 6.0f * (s_fC2Deriv1 - delta[n - 2]);
		solved = solveLinear(A, rhs, z);
	}
	else {
		z[0] = 0.0f;
		z[n - 1] = 0.0f;
		if (n == 3) {
			float diag = 2.0f * (h[0] + h[1]);
			z[1] = (diag != 0.0f) ? 6.0f * (delta[1] - delta[0]) / diag : 0.0f;
			solved = true;
		}
		else {
			int m = n - 2;
			std::vector<std::vector<float> > A(m, std::vector<float>(m, 0.0f));
			std::vector<float> rhs(m, 0.0f);
			for (int i = 0; i < m; ++i) {
				int k = i + 1;
				if (i > 0)
					A[i][i - 1] = h[k - 1];
				A[i][i] = 2.0f * (h[k - 1] + h[k]);
				if (i + 1 < m)
					A[i][i + 1] = h[k];
				rhs[i] = 6.0f * (delta[k] - delta[k - 1]);
			}
			std::vector<float> interior;
			solved = solveLinear(A, rhs, interior);
			if (solved) {
				for (int i = 0; i < m; ++i)
					z[i + 1] = interior[i];
			}
		}
	}

	if (!solved) {
		LinearCurveEvaluator linear;
		linear.evaluateCurve(ptvCtrlPts, ptvEvaluatedCurvePts, fAniLength, bWrap);
		return;
	}

	std::vector<Point> inside, wrapped;
	int spans = bWrap ? n : n - 1;
	for (int i = 0; i < spans; ++i) {
		int j = (i + 1) % n;
		float x1 = x[j];
		if (bWrap && j == 0)
			x1 = x[j] + fAniLength;
		sampleSpan(x[i], x1, y[i], y[j], z[i], z[j], s_iSegCount, fAniLength, inside, wrapped);
	}

	if (bWrap)
		ptvEvaluatedCurvePts = mergeWrapped(inside, wrapped, fAniLength);
	else {
		ptvEvaluatedCurvePts = inside;
		extendHorizontal(ptvEvaluatedCurvePts, fAniLength, y.front(), y.back());
	}
}
