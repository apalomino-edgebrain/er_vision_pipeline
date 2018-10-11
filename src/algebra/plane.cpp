#include "plane.h"
#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;
using namespace std;

bool line3d::ClosestPoint(const line3d &L1, const line3d &L2, Vector3d &Result1, Vector3d &Result2)
{
	Vector3d NewDir = L1.D.cross(L2.D);
	float Length = NewDir.norm();
	if (Length == 0.0f) {
		return false;
	}

	plane P1 = plane::fromPointVectors(L1.P0, NewDir, L1.D);
	plane P2 = plane::fromPointVectors(L2.P0, NewDir, L2.D);

	Result1 = P2.IntersectLine(L1);
	Result2 = P1.IntersectLine(L2);
	return true;
}

float line3d::Dist(const line3d &L1, const line3d &L2)
{
	Vector3d Cross = L1.D.cross(L2.D);
	Vector3d v = L2.P0 - L1.P0;

	return abs(v.dot(Cross)) / Cross.norm();
}

float line3d::DistSq(const line3d &L1, const line3d &L2)
{
	Vector3d Cross = L1.D.cross(L2.D);

	Vector3d v = L2.P0 - L1.P0;
	float Dot = v.dot(Cross);
	return (Dot * Dot) / Cross.squaredNorm();
}

float line3d::DistToPoint(const Vector3d &P) const
{
	float t0 = D.dot(P - P0) / D.dot(D);

	Vector3d v = P0 + t0 * D;

	// TODO: Replace this with the eigen equivalent
	float x = P.x() + v.x();
	float y = P.y() + v.y();
	float z = P.z() + v.z();

	float distanceLine = sqrt(x*x + y*y + z*z);
	return distanceLine;
}

//-----------------------------------------------------------------------------

plane::plane()
{

}

plane::plane(const plane &P)
{
	a = P.a;
	b = P.b;
	c = P.c;
	d = P.d;
}

plane::plane(float _a, float _b, float _c, float _d)
{
	a = _a;
	b = _b;
	c = _c;
	d = _d;
}

plane::plane(const Vector3d &NormalizedNormal, float _d)
{
	a = NormalizedNormal(0);
	b = NormalizedNormal(1);
	c = NormalizedNormal(2);
	d = _d;
}

plane plane::fromPointNormal(const Vector3d &Pt, const Vector3d &Normal)
{
	plane Result;
	Vector3d NormalizedNormal = Normal;
	NormalizedNormal.normalize();

	Result.a = NormalizedNormal(0);
	Result.b = NormalizedNormal(1);
	Result.c = NormalizedNormal(2);
	Result.d = -Pt.dot(NormalizedNormal);
	return Result;
}

plane plane::fromPointVectors(const Vector3d &Pt, const Vector3d &V1, const Vector3d &V2)
{
	Vector3d Normal = V1.cross(V2);
	return fromPointNormal(Pt, Normal);
}

plane plane::Normalize()
{
	plane Result;
	float Distance = sqrtf(a * a + b * b + c * c);
	Result.a = a / Distance;
	Result.b = b / Distance;
	Result.c = c / Distance;
	Result.d = d / Distance;
	return Result;
}

plane plane::fromPoints(const Vector3d &V0, const Vector3d &V1, const Vector3d &V2)
{
	Vector3d t0 = V1 - V0;
	Vector3d t1 = V2 - V0;
	Vector3d Normal = t0.cross(t1);
	Normal.normalize();
	return fromPointNormal(V0, Normal);
}

Vector3d plane::IntersectLine(const line3d &Line) const
{
	return IntersectLine(Line.P0, Line.P0 + Line.D);
}

Vector3d plane::IntersectLine(const Vector3d &V1, const Vector3d &V2) const
{
	Vector3d Diff = V1 - V2;
	float Denominator = a * Diff(0) + b * Diff(1) + c * Diff(2);
	if (Denominator == 0.0f) {
		return (V1 + V2) * 0.5f;
	}
	float u = (a * V1(0) + b * V1(1) + c * V1(2) + d) / Denominator;

	return (V1 + u * (V2 - V1));
}

Vector3d plane::IntersectLine(const Vector3d &V1, const Vector3d &V2, bool &Hit) const
{
	Hit = true;
	Vector3d Diff = V2 - V1;
	float denominator = a * Diff(0) + b * Diff(1) + c * Diff(2);
	if (denominator == 0) { Hit = false; return V1; }
	float u = (a * V1(0) + b * V1(1) + c * V1(2) + d) / denominator;

	return (V1 + u * (V2 - V1));
}

float plane::IntersectLineRatio(const Vector3d &V1, const Vector3d &V2)
{
	Vector3d Diff = V2 - V1;
	float Denominator = a * Diff(0) + b * Diff(1) + c * Diff(2);
	if (Denominator == 0.0f) {
		return 0.0f;
	}
	return (a * V1(0) + b * V1(1) + c * V1(2) + d) / -Denominator;
}

float plane::signedDistance(const Vector3d &Pt) const
{
	return (a * Pt(0) + b * Pt(1) + c * Pt(2) + d);
}

float plane::unsignedDistance(const Vector3d &Pt) const
{
	return abs(a * Pt(0) + b * Pt(1) + c * Pt(2) + d);
}

Vector3d plane::ClosestPoint(const Vector3d &Point)
{
	return (Point - Normal() * signedDistance(Point));
}

bool plane::planePlaneIntersection(const plane &P1, const plane &P2, line3d &L)
{
	float Denominator = P1.a * P2.b - P1.b * P2.a;
	if (Denominator == 0.0f) {
		// this case should be handled by switching axes...
		return false;
	}
	L.P0 = Vector3d((P2.d * P1.b - P1.d * P2.b) /
		Denominator, (P1.d * P2.a - P2.d * P1.a) /
		Denominator, 0.0f);

	L.D = P1.Normal().cross(P2.Normal());
	if (L.D.norm() == 0.0f) {
		return false;
	}
	L.D.normalize();

	return true;
}

float plane::Dot(const plane &P, const Vector4d &V)
{
	return P.a * V(0) + P.b * V(1) + P.c * V(2) + P.d * V(3);
}

float plane::DotCoord(const plane &P, const Vector3d &V)
{
	return P.a * V(0) + P.b * V(1) + P.c * V(2) + P.d;
}

float plane::DotNormal(const plane &P, const Vector3d &V)
{
	return P.a * V(0) + P.b * V(1) + P.c * V(2);
}

