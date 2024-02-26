#pragma once

#include "lidar_scan.h"

class LineSegment
{
public:
	static std::vector<Point> DouglasPeucker(std::vector<Point>& pointList, float epsilon);
private:
	static float perpendicularDistance(const Point& p, const Point& line_p1, const Point& line_p2);
};
