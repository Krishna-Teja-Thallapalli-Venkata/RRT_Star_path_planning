# RRT* Path Planning in 2D Grid

A complete C++ implementation of the RRT* (Rapidly-exploring Random Tree Star) algorithm for optimal path planning in a 2D grid environment with obstacles.

##  Prerequisites
*   **C++ Compiler:** A C++11 (or newer) compliant compiler (MSVC is used in this project).
*   **CMake:** Version 3.10 or higher.
*   **OpenCV:** Version 4.x is recommended. Ensure it's compiled/installed compatibly with your chosen C++ compiler.
*   **Git:** For cloning the repository.
*   **Doxygen:** For generating API documentation.


## Features

- **Complete RRT* Algorithm**: Implements core RRT* logic including random sampling, nearest neighbor search, steering, collision checking, choosing the best parent, and tree rewiring for optimality.
- **2D Grid Environment**: Represents a grid with definable obstacles.
- **Collision Detection**: Check for collisions between path segments and obstacles.
- **Path Smoothing**: Includes a basic post-processing step to shorten and straighten the found path.
- **Visualization**: Uses OpenCV to display the grid, obstacles, RRT* tree growth, start/end points, and the final path.
- **Configurable Parameters**: Allows adjustment of `step_size`, `search_radius`, and `max_iterations` for the RRT* algorithm.
- **Output**: Generates a visual representation of the planning result (`rrt_star_result.png` or `rrt_star_failed.png`) and a text file with path coordinates (`path_coordinates.txt`).
- **Modular Code**: Code is split into logical units (Point, Node, RRTStar, Visualizer) for better organization and maintainability.
- **CMake Build System**: Uses CMake for cross-platform building.
- **Doxygen Documentation**: Supports Doxygen for automatic API documentation generation.


## Project Structure
```text
RRT-Star_path_planning/
├── src/ # Source files.
│ ├── main.cpp # Main application entry point, setup, and demo logic
│ ├── point.h # Definition of the Point structure
│ ├── node.h # Definition of the Node structure
│ ├── rrt_star.h # RRTStar class declaration
│ ├── rrt_star.cpp # RRTStar class implementation
│ ├── visualizer.h # PathVisualizer class declaration
│ └── visualizer.cpp # PathVisualizer class implementation
├── build/ # Build output directory (created by CMake/build script)
├── CMakeLists.txt # CMake build script for the project
├── Doxyfile # Configuration file for Doxygen API documentation
├── build.bat # Windows batch script for easy building
├── README.md # Quick overview and setup instructions
├── .gitignore
```
## The RRT* Algorithm Explained

#### Overview
The RRT* algorithm incrementally builds a tree of collision-free paths starting from the `start_node`. It explores the configuration space by randomly sampling points and attempting to connect them to the existing tree. Unlike the basic RRT, RRT* continuously refines the tree structure to ensure that paths are not just feasible but also tend towards optimality (typically shortest path length).

#### Key Steps in RRT* (as implemented)

For a predefined number of `max_iterations` or until a satisfactory path to the end is found and refined:

1.  **Random Sample ( `generateRandomPoint` ):**
    *   Generate a random point (`random_point`) within the confines of the grid.
    *   A "goal bias" is introduced: occasionally (e.g., 10% of the time), `random_point` is set to the `end_pt` directly to encourage growth towards the end.

2.  **Find Nearest Node ( `findNearestNode` ):**
    *   Search the existing tree (`nodes`) to find the node (`nearest_node`) closest to `random_point`.

3.  **Steer ( `steer` ):**
    *   Generate a new point (`new_point`) by "steering" from `nearest_node.point` towards `random_point`.
    *   The `new_point` is at most `step_size` distance away from `nearest_node.point` along the direction of `random_point`. If `random_point` is closer than `step_size`, `new_point` becomes `random_point`.

4.  **Collision Check ( `isCollisionFree` ):**
    *   Verify if the straight-line path segment between `nearest_node.point` and `new_point` is free of obstacles. If a collision occurs, this iteration is aborted, and the process restarts from Step 1.

5.  **Find Near Nodes ( `findNearNodes` ):**
    *   Identify all existing nodes (`near_nodes`) in the tree that are within a `search_radius` of `new_point`. This set will be used for choosing the best parent and for rewiring.

6.  **Choose Parent (Core RRT* Step):**
    *   Initialize `best_parent` to `nearest_node`. The initial cost to reach `new_point` is `nearest_node.cost + distance(nearest_node.point, new_point)`.
    *   Iterate through all nodes in `near_nodes`. For each `near_node_candidate`:
        *   If a collision-free path exists from `near_node_candidate.point` to `new_point`:
            *   Calculate the cost to reach `new_point` via `near_node_candidate` (`near_node_candidate.cost + distance(near_node_candidate.point, new_point)`).
            *   If this cost is lower than the current best cost to `new_point`, update `best_parent` to `near_node_candidate` and update the minimum cost.
    *   Create a `new_node` at `new_point`. Set its parent to `best_parent` and its cost to the calculated minimum cost. Add `new_node` to the tree (`nodes`) and as a child of `best_parent`.

7. **Rewire Tree (Core RRTStar Step):**
    *   Iterate through all nodes in `near_nodes` (excluding `new_node`'s actual parent). For each `near_node_to_rewire`:
        *   If a collision-free path exists from `new_point` to `near_node_to_rewire.point`:
            *   Calculate the potential new cost for `near_node_to_rewire` if it were parented by `new_node` (`new_node.cost + distance(new_point, near_node_to_rewire.point)`).
            *   If this potential cost is lower than `near_node_to_rewire.cost`:
                *   Disconnect `near_node_to_rewire` from its old parent.
                *   Set `new_node` as the new parent of `near_node_to_rewire`.
                *   Update `near_node_to_rewire.cost` to the new, lower cost.
                *   Recursively update the costs of all descendants of `near_node_to_rewire` ( `updateCosts` function).

8.  **Check End Reached:**
    *   If `new_point` is within `step_size` of `end_pt` and the path from `new_point` to `end_pt` is collision-free:
        *   A potential path to the end has been found. A `end_node` is created (or updated if a previous, more costly path to end existed).
        *   The algorithm continues to run for the remaining iterations to potentially find even better paths to the end through the rewiring process.

The algorithm terminates after `max_iterations`. The best path found to `end_node` (if any) is then reconstructed.

## **Environment Representation**
The 2D environment is modeled as a grid (`std::vector<std::vector<bool>> grid`).
*   Each cell `grid[y][x]` can be `true` (obstacle) or `false` (free space).
*   Coordinates are typically treated as `double` for point calculations and then cast to `int` for grid lookups.
*   Points outside the grid boundaries are implicitly considered obstacles.

When run, the application will:
1.  Print setup parameters (grid size, start/end) to the console.
2.  Announce the start of RRT* planning and provide iteration progress updates.
3.  If a path is found:
    *   Report success, path lengths (original and smoothed), and total cost.
    *   Save path coordinates to `path_coordinates.txt`.
    *   Display the visualization using OpenCV.
    *   Save the visualization to `rrt_star_result.png`.
4.  If no path is found within `max_iterations`:
    *   Report failure.
    *   Display the explored tree (if any).
    *   Save the visualization to `rrt_star_failed.png`.
5.  The visualization window will wait for a key press before closing. The console application will wait for an Enter press before exiting.

##  Configuration Parameters
Currently, parameters are hardcoded in `src/main.cpp`. To change them, modify these values and recompile:

*   `GRID_WIDTH`, `GRID_HEIGHT`: Dimensions of the planning grid.
    *   Default: `400`x`300`
*   `Point start(x, y)`, `Point end(x, y)`: Start and end coordinates.
    *   Default: `start(10, 10)`, `end(380, 280)`
*   `RRTStar planner(...)`:
    *   `step_size`: Max distance to extend a new node (influences tree density and exploration speed).
        *   Default: `15.0`
    *   `search_radius`: Radius for finding near nodes for parent selection and rewiring (critical for RRT* optimality).
        *   Default: `30.0`
    *   `max_iterations`: Maximum number of iterations for the RRT* algorithm.
        *   Default: `8000` (increased from original to allow more refinement)
*   Obstacle definitions: The `createSampleEnvironment()` function in `src/main.cpp` defines the obstacle layout. This can be modified to create different scenarios.

##  Running the Application
The executable `rrt_star_planner.exe`  will be located in the `build/bin/Release/` directory.
*   Navigate to `build/bin/Release`.
*   Execute: `rrt_star_planner.exe`.

## Output Files
Generated in the directory from which the executable is run (typically `build/bin/`):
*   `rrt_star_result.png`: Image visualizing a successful planning result (tree, obstacles, path).
*   `rrt_star_failed.png`: Image visualizing the explored tree if planning fails.
*   `path_coordinates.txt`: A text file listing the (x, y) coordinates of the waypoints in the final smoothed path, one point per line. Example:
    ```
    # Path coordinates (x, y)
    10, 10
    55.3, 72.1
    ...
    380, 280
    ```

## Visualization

### Understanding the Visual Output
The OpenCV window displays:
*   **Background:** White, representing free traversable space.
*   **Obstacles:** Solid black rectangular areas.
*   **RRTStar Tree:** 
    *   **Edges:** Light gray lines connecting parent nodes to their children.
    *   **Nodes:** Small dark gray circles (drawing can be toggled in `visualizer.cpp` for clarity on large trees).
*   **Start Point:** A larger blue circle.
*   **end Point:** A larger red circle.
*   **Final Path:** A thicker green line representing the smoothed, optimal path found from start to end.

The visualization scale factor in `src/main.cpp` (`PathVisualizer visualizer(GRID_WIDTH, GRID_HEIGHT, 2);`) can be adjusted; a factor of `2` means each grid unit is drawn as 2x2 pixels, making details more visible.

Below is reference output:

![rrt_star_result](https://github.com/user-attachments/assets/aa442329-1eb3-421b-a8bc-e051ef682d23)


## Source Code Documentation (Doxygen)

The source code (`.h` and `.cpp` files in `src/`) includes Doxygen-compatible comments.
To generate HTML documentation:
1.  Ensure Doxygen is installed.
2.  Navigate to the project's root directory.
3.  Run the command: `doxygen Doxyfile`
4.  Open `doxygen_output/html/index.html` in a web browser.

The documentation will provide details on classes, methods, structures, and their parameters.

## Conclusion

This project successfully implements the RRT* algorithm for optimal path planning in a 2D grid environment. It demonstrates key concepts of randomized motion planning, including tree-based exploration, collision detection, and path optimization through parent selection and rewiring. The inclusion of path smoothing and OpenCV-based visualization provides a complete and understandable demonstration of the algorithm's capabilities. The modular C++ design and CMake build system make the project maintainable and extensible for future enhancements.
