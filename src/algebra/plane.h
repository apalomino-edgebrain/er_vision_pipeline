#include <Eigen/Core>

struct line3d
{
	line3d() {}
	line3d(const Eigen::Vector3d &_P0, const Eigen::Vector3d &_D)
	{
		P0 = _P0;
		D = _D;
	}

	float DistToPoint(const Eigen::Vector3d &V) const;

	static bool ClosestPoint(const line3d &L1, const line3d &L2, Eigen::Vector3d &Result1, Eigen::Vector3d &Result2);
	static float Dist(const line3d &L1, const line3d &L2);
	static float DistSq(const line3d &L1, const line3d &L2);

	Eigen::Vector3d P0;
	Eigen::Vector3d D;
};

struct plane
{
	//
	// Initalization
	//
	plane();
	plane(const plane &P);
	plane(float _a, float _b, float _c, float _d);
	plane(const Eigen::Vector3d &NormalizedNormal, float _d);

	// Returns the y point for an X and z
	float get_y(float x, float z);

	//
	// Static constructors
	//
	static plane fromPointNormal(const Eigen::Vector3d &Pt, const Eigen::Vector3d &Normal);
	static plane fromPointVectors(const Eigen::Vector3d &Pt, const Eigen::Vector3d &V1, const Eigen::Vector3d &V2);
	static plane fromPoints(const Eigen::Vector3d &V1, const Eigen::Vector3d &V2, const Eigen::Vector3d &V3);

	//
	// Math functions
	//
	float unsignedDistance(const Eigen::Vector3d &Pt) const;
	float signedDistance(const Eigen::Vector3d &Pt) const;
	//bool fitToPoints(const std::Vector<Eigen::Vector3d> &Points, float &ResidualError);
	//bool fitToPoints(const std::Vector<Eigen::Vector4d> &Points, Eigen::Vector3d &Basis1, Eigen::Vector3d &Basis2, float &NormalEigenvalue, float &ResidualError);
	Eigen::Vector3d ClosestPoint(const Eigen::Vector3d &Point);

	//
	// Line intersection
	//

	//determines the intersect of the line defined by the points V1 and V2 with the plane.
	Eigen::Vector3d IntersectLine(const Eigen::Vector3d &V1, const Eigen::Vector3d &V2) const;

	//Returns the point of intersection.  Origin is returned if no intersection exists.
	//determines the intersect of the line defined by the points V1 and V2 with the plane.
	Eigen::Vector3d IntersectLine(const Eigen::Vector3d &V1, const Eigen::Vector3d &V2, bool &Hit) const;

	//If there is no intersection, Hit will be false.
	//Paramaterize the line with the variable t such that t = 0 is V1 and t = 1 is V2.
	float IntersectLineRatio(const Eigen::Vector3d &V1, const Eigen::Vector3d &V2);

	//returns the t for this line that lies on this plane.
	Eigen::Vector3d IntersectLine(const line3d &Line) const;

	//dot product of a plane and a 4D vector
	static float Dot(const plane &P, const Eigen::Vector4d &V);

	//dot product of a plane and a 3D coordinate
	static float DotCoord(const plane &P, const Eigen::Vector3d &V);

	//dot product of a plane and a 3D normal
	static float DotNormal(const plane &P, const Eigen::Vector3d &V);

	//
	// Normalization
	//
	plane Normalize();

	//
	// Accessors
	//
	__forceinline Eigen::Vector3d plane::Normal() const
	{
		return Eigen::Vector3d(a, b, c);
	}

	__forceinline plane Flip()
	{
		plane Result;
		Result.a = -a;
		Result.b = -b;
		Result.c = -c;
		Result.d = -d;
		return Result;
	}

	static bool planePlaneIntersection(const plane &P1, const plane &P2, line3d &L);

	float a, b, c, d;        //the (a, b, c, d) in a*x + b*y + c*z + d = 0.
};
