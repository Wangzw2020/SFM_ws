#include "ros/ros.h"
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>
using namespace Eigen;

struct Point{
	float x;
	float y;
	float z;
};

struct Line{
	Point start;
	Point end;
};

struct Color{
	float r;
	float g;
	float b;
};

float randomFloat(float lower, float upper)
{
	return (lower + (static_cast<float>(rand()) / RAND_MAX) * (upper - lower));
}

Vector3d setVector(Point a, Point b)
{
	Vector3d v(b.x-a.x, b.y-a.y, b.z-a.z);
	return v;
}

Point setPoint(float x, float y, float z)
{
	Point p;
	p.x = x;
	p.y = y;
	p.z = z;
	return p;
}
