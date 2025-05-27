#include <iostream>
#include "rrt_star.h"
#include <algorithm>
#include <cmath>

using namespace std;

RRTStar::RRTStar(int width, int height, Point start_pt, Point end_pt, double step, double radius, int max_itr)
        : grid_width(width), grid_height(height), start_pt(start_pt), end_pt(end_pt), step_size(step), search_radius(radius),
        max_iterations(max_itr), gen(rd()), dis_x(0, width), dis_y(0, height), start_node(nullptr), end_node(nullptr){

            grid.resize(height, vector<bool>(width, false));
            start_node = new Node(start_pt);
            start_node->cost = 0.0; // starting cost
            nodes.push_back(start_node);

        }


RRTStar::~RRTStar(){
    for(Node* x: nodes){
        delete x;
    }
    nodes.clear();
}


void RRTStar::setObstacle(int x, int y){
    if(x>=0 && x<grid_width && y>=0 && y<grid_height){
        grid[y][x] = true;
    }
}

bool RRTStar::isObstacle(const Point& p) const{
    int x = p.x;
    int y = p.y;

    if(x<0 || x >=grid_width || y<0 || y>=grid_height){
        return true;
    }

    return grid[y][x];
}

bool RRTStar::isCollisionFree(const Point& p1, const Point& p2) const{
    double dist = p1.distance(p2);

    int steps = max(1, static_cast<int>(dist/2.0));

    for(int i=0;i<=steps;++i){
        double t = (steps == 0) ? 0.0 : static_cast<double>(i)/steps;

        Point check_point(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));

        if(isObstacle(check_point)){
            return false;
        }
    }

    return true;

}

Point RRTStar::generateRandomPoint(){
    return Point(dis_x(gen), dis_y(gen));
}

Node* RRTStar::findNearestNode(const Point& point){
    if(nodes.empty()){
        return nullptr;
    }

    Node* nearest = nodes[0];
    double min_dist = point.distance(nearest->point);

    for(size_t i = 0; i<nodes.size(); ++i){
        double dist = point.distance(nodes[i]->point);
        if(dist < min_dist){
            min_dist = dist;
            nearest = nodes[i];
        }

    }

    return nearest;

}

vector<Node*> RRTStar::findNearNodes(const Point& point){
    vector<Node*> near_nodes;
    for(Node* x: nodes){
        if(point.distance(x->point) <= search_radius){
            near_nodes.push_back(x);
        }
    }

    return near_nodes;
}

Point RRTStar::steer(const Point& from, const Point& to){
    double dist = from.distance(to);
    if(dist < step_size){
        return to;
    }

    double theta = atan2(to.y - from.y, to.x - from.x);

    return Point(from.x + step_size*cos(theta), from.y + step_size*sin(theta));
}

bool RRTStar::planPath(){
    cout << "Starting Path Planning.... " << endl;

    for(int iter = 0; iter < max_iterations; ++iter){
        if(iter%500 == 0){
            cout << "Iteration: " << iter << " / " << max_iterations << " (Nodes: " << nodes.size() << ")" << endl;
        }

        Point random_point;
        if(iter%10 == 0 && iter>0){
            random_point = end_pt;
        }
        else{
            random_point = generateRandomPoint();
        }

        Node* nearest_node = findNearestNode(random_point);
        if(!nearest_node) continue;
        
        Point new_point = steer(nearest_node->point, random_point);

        if (!isCollisionFree(nearest_node->point, new_point)) {
            continue;
        }

        Node* best_parent = nearest_node;
        double min_cost_to_new_point = nearest_node->cost + nearest_node->point.distance(new_point);

        vector<Node*> near_nodes = findNearNodes(new_point);
        if(near_nodes.empty()){
            near_nodes.push_back(nearest_node);
        }

        for (Node* x : near_nodes) {
            if (isCollisionFree(x->point, new_point)) {
                double cost_via_near_node = x->cost + x->point.distance(new_point);
                if (cost_via_near_node < min_cost_to_new_point) {
                    min_cost_to_new_point = cost_via_near_node;
                    best_parent = x;
                }
            }
        }

        // Create and connect new node
        Node* new_node = new Node(new_point);
        new_node->parent = best_parent;
        new_node->cost = min_cost_to_new_point;
        best_parent->children.push_back(new_node);
        nodes.push_back(new_node);

        for (Node* x : near_nodes) {
           
            if (x == best_parent || x == new_node) continue; 

            if (isCollisionFree(new_point, x->point)) {
                double cost_via_new_node = new_node->cost + new_point.distance(x->point);
                if (cost_via_new_node < x->cost) {
                    Node* old_parent = x->parent;
                    if (old_parent) { 
                        auto& siblings = old_parent->children;
                        siblings.erase(remove(siblings.begin(), siblings.end(), x), siblings.end());
                    }
                    
                    x->parent = new_node;
                    x->cost = cost_via_new_node;
                    new_node->children.push_back(x);
                    updateCosts(x); // Update costs of its descendants
                }
            }
        }

        // Check if end is reached
        if (new_point.distance(end_pt) <= step_size && isCollisionFree(new_point, end_pt)) {
             // Check if current path to end is better than existing end_node (if any)
            double cost_to_end = new_node->cost + new_point.distance(end_pt);
            if (!end_node || cost_to_end < end_node->cost) {
                if (end_node) { 
                    Node* old_end_parent = end_node->parent;
                    if(old_end_parent) {
                         old_end_parent->children.erase(remove(old_end_parent->children.begin(), old_end_parent->children.end(), end_node), old_end_parent->children.end());
                    }
                    auto it_end = find(nodes.begin(), nodes.end(), end_node);
                    if (it_end != nodes.end()) {
                        delete *it_end; // Delete the node object
                        nodes.erase(it_end); // Remove pointer from list
                    }
                }

                Node* final_end_node = new Node(end_pt);
                final_end_node->parent = new_node;
                final_end_node->cost = cost_to_end;
                new_node->children.push_back(final_end_node);
                nodes.push_back(final_end_node);
                end_node = final_end_node;
                
                cout << "End potentially reached/improved at iteration: " << iter << " with cost: " << end_node->cost << endl;
            }
        }


    }

    if (end_node) {
        cout << "Path planning finished. End reached with cost: " << end_node->cost << endl;
        return true;
    }

    cout << "Failed to reach end within " << max_iterations << " iterations." << endl;
    return false;

}


void RRTStar::updateCosts(Node* node) {
    for (Node* x : node->children) {
        x->cost = node->cost + x->point.distance(node->point);
        updateCosts(x); 
    }
}

vector<Point> RRTStar::getPath() {
    std::vector<Point> path;
    if (!end_node) {
        return path; // No path found
    }

    Node* current = end_node;
    while (current != nullptr) {
        path.push_back(current->point);
        current = current->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<Node*> RRTStar::getNodes() const {
    return nodes;
}

vector<Point> RRTStar::smoothPath(const vector<Point>& original_path) {
    if (original_path.size() <= 2) {
        return original_path;
    }

    vector<Point> smoothed_path;
    smoothed_path.push_back(original_path[0]);

    size_t i = 0;
    while (i < original_path.size() - 1) {
        size_t j = original_path.size() - 1;
        while (j > i + 1 && !isCollisionFree(original_path[i], original_path[j])) {
            j--;
        }
        smoothed_path.push_back(original_path[j]);
        i = j;
    }
    return smoothed_path;
}