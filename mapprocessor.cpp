/*
A* implementation in Cpp - Map Processor Node
This node currently converts raw lidar data from one scan of the robot into a 2D map.
*/

#include <cmath>
#include <stdio.h>
#include <vector>
#include <mapprocessor.hpp>

// Convert laser scan data to 2D points
std::vector<Point2D> scanToPoints(const std::vector<float>& ranges,
                                  float angle_min, float angle_increment) {
    std::vector<Point2D> pts;
    for (size_t k=0; k<ranges.size(); ++k) {
        float r = ranges[k];
        if (r <= 0) continue; // ignore invalid
        float theta = angle_min + k * angle_increment;
        Point2D p;
        p.x = r * cos(theta);  // forward
        p.y = r * sin(theta);  // left
        pts.push_back(p);
    }
    return pts;
}
