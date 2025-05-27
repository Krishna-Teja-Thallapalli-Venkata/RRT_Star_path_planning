#include "rrt_star.h"
#include "visualizer.h"
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

/**
 * @brief Sets up a sample environment with predefined obstacles for the RRT* planner.
 * 
 */

void createSampleEnvironment(RRTStar& planner) {
    // Obstacle 1: Rectangle in upper left
    for (int x = 20; x < 80; ++x) {
        for (int y = 20; y < 60; ++y) {
            planner.setObstacle(x, y);
        }
    }

    // Obstacle 2: Rectangle in middle
    for (int x = 120; x < 180; ++x) {
        for (int y = 80; y < 140; ++y) {
            planner.setObstacle(x, y);
        }
    }

    // Obstacle 3: L-shaped obstacle
    for (int x = 220; x < 280; ++x) {
        for (int y = 40; y < 100; ++y) {
            planner.setObstacle(x, y);
        }
    }
    for (int x = 220; x < 260; ++x) {
        for (int y = 100; y < 160; ++y) {
            planner.setObstacle(x, y);
        }
    }

    // Obstacle 4: Small scattered obstacles
    for (int x = 50; x < 150; x += 20) {
        for (int y = 180; y < 200; ++y) {
            planner.setObstacle(x, y);
        }
    }
}

/**
 * @brief Saves the path coordinates to the file.
 * 
 */

void savePathToFile(const vector<Point>& path, const string& filename) {
    ofstream file(filename);
    if (file.is_open()) {
        file << " Path coordinates (x, y)" << endl;
        for (const Point& p : path) {
            file << p.x << ", " << p.y << endl;
        }
        file.close();
        cout << "Path saved to " << filename << endl;
    } else {
        cerr << "Error: Unable to open file " << filename << " for writing." << endl;
    }
}

int main(int argc, char* argv[]) {
    (void)argc; 
    (void)argv; 

    cout << "----- RRT* Path Planning Demo -------" << endl;

    // Environment parameters
    const int GRID_WIDTH = 400;
    const int GRID_HEIGHT = 300;

    // Start and end points
    Point start(10, 10);
    Point end(380, 280);

    cout << "Grid size: " << GRID_WIDTH << " x " << GRID_HEIGHT << endl;
    cout << "Start: (" << start.x << ", " << start.y << ")" << endl;
    cout << "End: (" << end.x << ", " << end.y << ")" << endl;

    // RRT* parameters
    double step_size = 15.0;
    double search_radius = 30.0; 
    int max_iterations = 8000;  

    // Create RRT* planner
    RRTStar planner(GRID_WIDTH, GRID_HEIGHT, start, end, step_size, search_radius, max_iterations);

    // Create environment with obstacles
    cout << "Creating environment with obstacles..." << endl;
    createSampleEnvironment(planner);

    // Plan path
    bool success = planner.planPath();

    if (success) {
        cout << "Path found successfully!" << endl;

        vector<Point> original_path = planner.getPath();
        cout << "Original path length: " << original_path.size() << " points" << endl;

        vector<Point> smoothed_path = planner.smoothPath(original_path);
        cout << "Smoothed path length: " << smoothed_path.size() << " points" << endl;

        double total_cost = 0.0;
        if (!smoothed_path.empty()) {
            total_cost = planner.getNodes().back()->cost; // Cost from end node
        }
        cout << "Total path cost (from RRT* end_node): " << total_cost << endl;


        savePathToFile(smoothed_path, "path_coordinates.txt");

        cout << "Creating visualization..." << endl;
        PathVisualizer visualizer(GRID_WIDTH, GRID_HEIGHT, 2); 

        for (int x = 0; x < GRID_WIDTH; ++x) {
            for (int y = 0; y < GRID_HEIGHT; ++y) {
                if (planner.isObstacle(Point(static_cast<double>(x), static_cast<double>(y)))) {
                    visualizer.drawObstacle(x, y);
                }
            }
        }
        visualizer.drawTree(planner.getNodes());
        visualizer.drawPath(smoothed_path);
        visualizer.drawStartEnd(start, end);

        visualizer.save("rrt_star_result.png");
        visualizer.show();

    } else {
        cout << "Failed to find path!" << endl;
        PathVisualizer visualizer(GRID_WIDTH, GRID_HEIGHT, 2);
         for (int x = 0; x < GRID_WIDTH; ++x) {
            for (int y = 0; y < GRID_HEIGHT; ++y) {
                if (planner.isObstacle(Point(static_cast<double>(x), static_cast<double>(y)))) {
                    visualizer.drawObstacle(x, y);
                }
            }
        }
        visualizer.drawTree(planner.getNodes()); 
        visualizer.drawStartEnd(start, end);
        visualizer.save("rrt_star_failed.png");
        visualizer.show();
    }

    cout << "Program completed. Press Enter to exit..." << endl;
    cin.get();

    return 0;
}
