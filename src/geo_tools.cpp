// Tools to calculte distances and convert positions into 3D space.

#define _USE_MATH_DEFINES

// Examples https://github.com/nlohmann/json#examples
#include "json/json.hpp"

//#############################################################################
//######################### HAVERSINE FORMULA #################################
//#############################################################################

#include <math.h>
#include <cmath>
#define earthRadiusKm 6371.0

// This function converts decimal degrees to radians
inline double deg2rad(double deg)
{
	return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
inline double rad2deg(double rad)
{
	return (rad * 180 / M_PI);
}

/**
* Returns the distance between two points on the Earth.
* Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
* @param lat1d Latitude of the first point in degrees
* @param lon1d Longitude of the first point in degrees
* @param lat2d Latitude of the second point in degrees
* @param lon2d Longitude of the second point in degrees
* @return The distance between the two points in kilometers
*/
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d)
{
	double lat1r, lon1r, lat2r, lon2r, u, v;
	lat1r = deg2rad(lat1d);
	lon1r = deg2rad(lon1d);
	lat2r = deg2rad(lat2d);
	lon2r = deg2rad(lon2d);
	u = sin((lat2r - lat1r) / 2);
	v = sin((lon2r - lon1r) / 2);
	return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

// ER = Navigatigation

double distanceEarth_er(double lat1d, double lon1d, double lat2d, double lon2d)
{
	double dlong = deg2rad(lon2d - lon1d);
	double dlat = deg2rad(lat2d - lat1d);
	double a = pow(sin(dlat / 2.0), 2)
		+ cos(deg2rad(lat1d))
		* cos(deg2rad(lat2d))
		* pow(sin(dlong / 2.0), 2);

	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = earthRadiusKm * c * 1000.0;

	//cout << "Starting point: " << lat1d << " " << lon1d << endl;
	//cout << "Destination point: " << lat2d << " " << lon2d << endl;
	//cout << "Overall distance: " << d << endl;

	return d;
}