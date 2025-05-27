#ifndef POINT_H
#define POINT_H

#include <cmath>

using namespace std;

/**
 * @brief Struct to represent 2D point
 */
struct Point{
    double x;
    double y;

    /**
     * @brief Default constructor for Point it initialize with (0,0)
     */
    Point() :x(0),y(0) {}

    /**
     * @brief Constructor that takes arguments to initialize the point
     * @param x_coord The x-coordinate
     * @param y_coord The y-coordinate
     */
    Point(double x_coord, double y_coord) :x(x_coord),y(y_coord) {}

    /**
     * @brief Caluclate the eucledian distance b/w 2 points
     * 
     */
    double distance(const Point& other) const{
        return sqrt(pow((x - other.x), 2) + pow((y - other.y), 2));
    }
    
    /**
     * @brief Check if both points are equal
     */
    bool operator==(const Point& other) const {
        return (x == other.x && y == other.y);
    }

};

#endif 