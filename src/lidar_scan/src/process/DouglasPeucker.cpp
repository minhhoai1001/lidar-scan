#include "DouglasPeucker.h"

std::vector<Point> LineSegment::DouglasPeucker(std::vector<Point>& pointList, float epsilon)
{
	std::vector<Point> resultList;

	// Find the point with the maximum distance
	float dmax = 0;
	int index = 0;
	for (unsigned int i = 1; i < pointList.size() - 1; ++i)
	{
		float d = perpendicularDistance(pointList[i], pointList[0], pointList[pointList.size() - 1]);  
		if (d > dmax) {
			index = i;
			dmax = d;
		}
	}

	// If max distance is greater than epsilon, recursively simplify
	if (dmax > epsilon)
	{
		std::vector<Point> pre_part, next_part;
		for (int i = 0; i <= index; ++i)	pre_part.push_back(pointList[i]);
		for (unsigned int i = index; i < pointList.size(); ++i)	next_part.push_back(pointList[i]);
		// Recursive call
		std::vector<Point> resultList1 = DouglasPeucker(pre_part, epsilon);
		std::vector<Point> resultList2 = DouglasPeucker(next_part, epsilon);

		// combine
		resultList.insert(resultList.end(), resultList1.begin(), resultList1.end());
		resultList.insert(resultList.end(), resultList2.begin()+1, resultList2.end());
	}
	else
	{
		resultList.push_back(pointList[0]);
		resultList.push_back(pointList[pointList.size() - 1]);
	}
		
	return resultList;
}

float LineSegment::perpendicularDistance(const Point& p, const Point& line_p1, const Point& line_p2)
{
	Point vec1{p.x - line_p1.x, p.y - line_p1.y};
	Point vec2{line_p2.x - line_p1.x, line_p2.y - line_p1.y};
	float d_vec2 = sqrt(vec2.x*vec2.x + vec2.y*vec2.y);
	float cross_product = vec1.x*vec2.y - vec2.x*vec1.y;
	float d = (cross_product / d_vec2)<0 ? (cross_product / d_vec2)*-1: (cross_product / d_vec2);
	return d;
}