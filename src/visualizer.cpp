#include "visualizer.h"

using namespace std;

PathVisualizer::PathVisualizer(int width, int height, int scale_factor) : scale(scale_factor){
    image = cv::Mat::zeros(height * scale, width * scale, CV_8UC3);
    image.setTo(cv::Scalar(255,255,255)); //white background
}

void PathVisualizer::drawObstacle(int x, int y) {
    cv::rectangle(image,
                  cv::Point(x * scale, y * scale),
                  cv::Point((x + 1) * scale -1, (y + 1) * scale -1), // -1 for grid lines
                  cv::Scalar(0, 0, 0), -1); // Black obstacles
}

void PathVisualizer::drawTree(const vector<Node*>& nodes) {
    // Draw tree edges
    for (const Node* t : nodes) {
        if (t->parent != nullptr) {
            cv::line(image,
                     cv::Point(static_cast<int>(t->parent->point.x * scale), static_cast<int>(t->parent->point.y * scale)),
                     cv::Point(static_cast<int>(t->point.x * scale), static_cast<int>(t->point.y * scale)),
                     cv::Scalar(200, 200, 200), 1); // Light gray tree
        }
    }

    // Draw nodes (optional, can be cluttered for large trees)
    /*
    for (const Node* t : nodes) {
        cv::circle(image,
                   cv::Point(static_cast<int>(t->point.x * scale), static_cast<int>(t->point.y * scale)),
                   1, cv::Scalar(150, 150, 150), -1); // Darker gray nodes
    }
    */
}

void PathVisualizer::drawPath(const vector<Point>& path) {
    if (path.size() < 2) return;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::line(image,
                 cv::Point(static_cast<int>(path[i].x * scale), static_cast<int>(path[i].y * scale)),
                 cv::Point(static_cast<int>(path[i + 1].x * scale), static_cast<int>(path[i + 1].y * scale)),
                 cv::Scalar(0, 255, 0), 2); // Green path, thickness 2
    }
}

void PathVisualizer::drawStartEnd(const Point& start, const Point& end) {
    // Draw start point (blue)
    cv::circle(image,
               cv::Point(static_cast<int>(start.x * scale), static_cast<int>(start.y * scale)),
               std::max(3, scale * 2), cv::Scalar(255, 0, 0), -1); // Blue, radius based on scale

    // Draw end point (red)
    cv::circle(image,
               cv::Point(static_cast<int>(end.x * scale), static_cast<int>(end.y * scale)),
               std::max(3, scale * 2), cv::Scalar(0, 0, 255), -1); // Red, radius based on scale
}

void PathVisualizer::show(const string& window_name) {
    cv::imshow(window_name, image);
    cv::waitKey(0);
    cv::destroyAllWindows();
}

void PathVisualizer::save(const string& filename) {
    cv::imwrite(filename, image);
    std::cout << "Visualization saved as '" << filename << "'" << std::endl;
}