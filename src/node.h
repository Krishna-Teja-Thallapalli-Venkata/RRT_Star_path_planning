#ifndef NODE_H
#define NODE_H

#include "point.h"
#include <vector>
#include <limits> // for the numeric limits

using namespace std;

/**
 * @brief Node structer for RRT*
 */
struct Node{
    Point point;
    Node* parent;

    vector<Node*> children;
    double cost;

    /**
     * @brief Constructor for node initialization
     */
    Node(Point p) : point(p), parent(nullptr), cost(numeric_limits<double>::infinity()) {}
    
    /**
     * @brief  Constructor for Node with parent and cost
     */
    Node(Point p, Node* parent_node, double c) : point(p), parent(parent_node), cost(c){}

};

#endif