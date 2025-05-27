#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "point.h"
#include "node.h" 
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

/**
 * @brief Class for visualizing the RRT* path planning process using OpenCV.
 */
class PathVisualizer{
private:
    cv::Mat image;
    int scale;

public:

    /**
     * @brief Constructor of PathVisualizer
     */
    PathVisualizer(int width, int height, int scale_factor = 1);
    
    /**
     * @brief draws a obstacle
     */
    void drawObstacle(int x, int y);

    /**
     * @brief draws a tree
     */
    void drawTree(const vector<Node*>& nodes);

    /**
     * @brief draws the planned path
     */
    void drawPath(const vector<Point>& path);

    /**
     * @brief draws th start to end points
     */
    void drawStartEnd(const Point& start, const Point& end);

    /**
     * @brief shows the visualization in image
     */
    void show(const string& window_name = "RRT* Path Planning");

    /**
     * @brief saves the visualization
     */
    void save(const string& filename);

};

#endif