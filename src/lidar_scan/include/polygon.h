#ifndef _POLYGON_H_
#define _POLYGON_H_

#include "lidar_scan.h"
#include <bits/stdc++.h>

class Polygon
{
private:
    struct line 
    {
        Point p1, p2;
    };
    std::vector<Point> polygon;
    bool onLine(line* l1, Point* p);
    int direction(Point* a, Point* b, Point* c);
    bool isIntersect(line* l1, line* l2);

public:
    Polygon(std::vector<Point> points);
    bool checkInside( Point* p);
};

#endif