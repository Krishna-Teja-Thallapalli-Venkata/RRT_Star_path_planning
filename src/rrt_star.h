#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "point.h"
#include "node.h"
#include <vector>
#include <random>

using namespace std;

class RRTStar {
private:

    int grid_width, grid_height;
    vector<vector<bool>> grid; // 1-No obstacle 0-obstacle
    Point start_pt, end_pt;
    double step_size;
    double search_radius;
    int max_iterations;


    vector<Node*> nodes;
    Node* start_node;
    Node* end_node;

    random_device rd;
    mt19937 gen;
    uniform_real_distribution<double> dis_x, dis_y;

    /**
     * @brief generates a random point
     */
    Point generateRandomPoint();

    /**
     * @brief find nearest node in tree
     */
    Node* findNearestNode(const Point& point);

    /**
     * @brief finds all the nodes in the search radius
     */
    vector<Node*> findNearNodes(const Point& point);

    /**
     * @brief steer from present point to end point
     */
    Point steer(const Point& from, const Point& to);

    /**
     * @brief update costs from start node
     */
    void updateCosts(Node* node);

public:

    RRTStar(){}

    /**
     * @brief Constructor for path planner
     */
    RRTStar(int width, int height, Point start_pt, Point end_pt, double step=10.0, double radius=20.0, int max_itr = 5000);

    /**
     * @brief Destructor
     */
    ~RRTStar();

    /**
     * @brief setting obstacle
     */
    void setObstacle(int x, int y);

    /**
     * @brief Check if point is obstacle or not?
     */
    bool isObstacle(const Point& p) const;

    /**
     * @brief check if line between 2 points doesn't have collision 
     */
    bool isCollisionFree(const Point& p1, const Point& p2) const;

    /**
     * @brief plans the path from start to end
     */
    bool planPath();

    /**
     * @brief get the plan from start to end
     */
    vector<Point> getPath();

    /**
     * @brief gets all the nodes in tree
     */
    vector<Node*> getNodes() const;

    /**
     * @brief smooths the path
     */
    vector<Point> smoothPath(const vector<Point>& original_path);

};

#endif