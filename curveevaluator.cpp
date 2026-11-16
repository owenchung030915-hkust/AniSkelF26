#include "CurveEvaluator.h"

float CurveEvaluator::s_fFlatnessEpsilon = 0.25f;
int CurveEvaluator::s_iSegCount = 16;
bool CurveEvaluator::s_bAdaptiveBezier = false;
int CurveEvaluator::s_iBezierContinuity = 0;
float CurveEvaluator::s_fCatmullTension = 0.0f;
int CurveEvaluator::s_iBSplineDegree = 3;
bool CurveEvaluator::s_bLaneRiesenfeld = false;
bool CurveEvaluator::s_bC2Clamped = false;
float CurveEvaluator::s_fC2Deriv0 = 0.0f;
float CurveEvaluator::s_fC2Deriv1 = 0.0f;
int CurveEvaluator::s_iLastPointCount = 0;
int CurveEvaluator::s_iOptionGeneration = 0;

CurveEvaluator::~CurveEvaluator(void)
{
}
