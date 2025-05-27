# RRT* Path Planning in 2D Grid

A complete C++ implementation of the RRT* (Rapidly-exploring Random Tree Star) algorithm for optimal path planning in a 2D grid environment with obstacles.

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
