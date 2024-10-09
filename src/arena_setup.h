#pragma once
#include <iostream>
#include <vector>
#include <memory>
#include <queue>
#include <cmath>
#include <random>
#include <unordered_set>
#include <mutex>
#include <thread>
#include <chrono>
#include <ctime>
#include <sstream>
#include <condition_variable>
#include <SFML/Graphics.hpp>
//#include <prometheus/exposer.h>
//#include <prometheus/registry.h>
//#include <prometheus/gauge.h>
//#include <prometheus/counter.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h> // To log to a file
#include <iomanip>

extern std::shared_ptr<spdlog::logger> logger;
std::shared_ptr<spdlog::logger> logger;  // Declare the logger globally

// Templated Point structure for 2D and 3D space
template <typename T>
class Point {
    T x, y, z;
public:
    Point() : x(0), y(0), z(0) {}
    Point(T x, T y) : x(x), y(y), z(0) {}
    Point(T x, T y, T z) : x(x), y(y), z(z) {}

    T getX() const { return x; }
    T getY() const { return y; }
    T getZ() const { return z; }
    void modify_x(T x) { this->x = x; } 
    void modify_y(T y) { this->y = y; }
    void modify_z(T z) { this->z = z; }
    void print() const 
    {
        std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
    }

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

template <typename T>
struct PointHash {
    std::size_t operator()(const Point<T>& p) const {
        return std::hash<T>()(p.getX()) ^ (std::hash<T>()(p.getY()) << 1) ^ (std::hash<T>()(p.getZ()) << 2);
    }
};

template <typename T>
struct PointEqual {
    bool operator()(const Point<T>& lhs, const Point<T>& rhs) const {
        return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY() && lhs.getZ() == rhs.getZ();
    }
};

// Templated Robot class    
template <typename T>
class Robot {
    T length, width, height;
public:
    Robot() : length(0), width(0), height(0) {} 
    Robot(T length, T width) : length(length), width(width), height(0) {} 
    Robot(T length, T width, T height) : length(length), width(width), height(height) {} 
    int getlength() const {
        return length;
    }
    int getwidth() const {
        return width;
    }
    int getheight() const {
        return height;
    }
};

// Node structure for RRT tree
template <typename T>
class Node {
    Point<T> point;
    Node* parent;
    std::vector<std::unique_ptr<Node<T>>> children;
public:
    Node(Point<T> point, Node* parent = nullptr) 
        : point(point), parent(parent) {}

    const Point<T>& getPoint() const { return point; }
    Node* getParent() const { return parent; }

    void addChild(std::unique_ptr<Node<T>> child) {
        children.push_back(std::move(child));
    }

    const std::vector<std::unique_ptr<Node<T>>>& getChildren() const {
        return children;
    }
};

// Setup class with arena configuration
template <typename T>
class Setup {
public:
    Robot<T> robot;
    T length, width, height, step_size,dim;
    std::vector<std::vector<int>> arena;
    Point<T> start, target;
    std::unordered_set<Point<T>, PointHash<T>, PointEqual<T>> points;

    Setup(T l,T w,T length, T width, Point<T> start, Point<T> target, T step_size)
        :robot(l,w), length(length), width(width), start(start), target(target), dim(std::max(l,w)),step_size(step_size) {
        arena.resize(std::ceil(width / dim), std::vector<int>(std::ceil(length / dim), 0));
        
        // Logger print statements
        logger->info("Initializing Setup...");
        logger->debug("Length: {}, Width: {}", length, width);
        logger->debug("Start Point: ({}, {})", start.getX(), start.getY());
        logger->debug("Target Point: ({}, {})", target.getX(), target.getY());
        logger->debug("Robot Length: {}, Robot Width: {}", l, w);
        logger->debug("Step Size: {}", step_size);
        logger->debug("Arena Dimensions: {}x{}", arena.size(), arena[0].size());
        logger->debug("Unit cell size: {}", dim);
        logger->info("Setup complete.");
    }

    bool isValid(const Point<T>& point) const {
        int x = static_cast<int>(point.getX() / dim);
        int y = static_cast<int>(point.getY() / dim);
        return x >= 0 && x < std::ceil(length / dim) && y >= 0 && y < std::ceil(width/ dim) && arena[y][x] == 0;
    }

    void markCell(const Point<T>& point, int value) {
    int x = static_cast<int>(point.getX() / dim);
    int y = static_cast<int>(point.getY() / dim);
    
    if (x >= 0 && x < arena[0].size() && y >= 0 && y < arena.size()) 
    {
            arena[y][x] = value;
            logger->debug("Marked cell at ({}, {}) with value {}", x, y, value);
    }
   else 
    {
    logger->warn("Tried to mark cell out of bounds at ({}, {})", x, y);
    }
    }

    void addObstacle(const std::vector<Point<T>>& obstacle);
};

// RRTPlanner with multi-threading and explicit functions
// RRTPlanner with multi-threading and explicit functions
template <typename T>
class RRTPlanner {
    std::mt19937 gen;
    std::uniform_real_distribution<> distX, distY;
    Setup<T>& setup;
    std::unique_ptr<Node<T>> root;
    std::mutex treeMutex;
    std::condition_variable cv;
    bool targetReached = false;
    int count;

public: // Add this to declare public members
    RRTPlanner(Setup<T>& setup) 
        : gen(std::random_device{}()), distX(0, setup.length), distY(0, setup.width), count(1), setup(setup),
        root(std::make_unique<Node<T>>(setup.start)) {
        setup.markCell(setup.start, 1);
    }
    std::unique_ptr<Node<T>> getRoot() { return std::move(root); }

    // Nearest neighbor search
    Node<T>* findNearest(const Point<T>& randomPoint, Node<T>* currentNode, double& minDistance);

    // Function to sample random points
    Point<T> samplePoint();

    // Check if the path between two points is clear
    bool collision_avoidance_check(Point<T>& randomPoint, const Point<T>& nearestPoint);

    // Add a new node to the tree
    void addNode(Node<T>* nearestNode, const Point<T>& newPoint);  // Ensure this is public

    // Main RRT loop for the thread
    void run(int thread_id);
    std::vector<Point<T>> getShortestPath();
    // Start the RRT planner with multiple threads
    void start(int num_threads);

private:  // If you have private members or helper functions, declare them here
    double calculateDistance(const Point<T>& p1, const Point<T>& p2) {
        return std::sqrt(std::pow(p1.getX() - p2.getX(), 2) + std::pow(p1.getY() - p2.getY(), 2));
    }
};


// Function to visualize the RRT tree, obstacles, and the start and target points

// Function to visualize the RRT tree, obstacles, and the start and target points
template <typename T>
void visualize(const Setup<T>& setup, const Node<T>* root, const std::vector<Point<T>>& path) {
    // Create a window for visualization
    sf::RenderWindow window(sf::VideoMode(setup.length, setup.width), "RRT Tree and Path Visualization");

    // Define colors
    sf::Color obstacleColor = sf::Color(101, 67, 33);   // Dark Brown for obstacles
    sf::Color nodeColor = sf::Color(0, 128, 128);       // Dark Teal for nodes
    sf::Color lineColor = sf::Color(0, 102, 102);       // Slightly lighter Teal for lines between nodes
    sf::Color startEndColor = sf::Color(50, 50, 50);    // Dark Gray for start and end points
    sf::Color pathColor = sf::Color::Black;             // Black for the path

    // Set path thickness
    float pathThickness = 6.0f;  // Increase this to make the path more prominent

    // Run the visualization loop
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);  // Clear with white background

        // Draw obstacles (as brown rectangles)
        for (int i = 0; i < setup.arena.size(); ++i) {
            for (int j = 0; j < setup.arena[0].size(); ++j) {
                if (setup.arena[i][j] == -1) {
                    sf::RectangleShape obstacle(sf::Vector2f(setup.dim, setup.dim));
                    obstacle.setPosition(j * setup.dim, i * setup.dim);
                    obstacle.setFillColor(obstacleColor);
                    window.draw(obstacle);
                }
            }
        }

        // Function to recursively draw the nodes and lines
        std::function<void(const Node<T>*)> drawTree = [&](const Node<T>* node) {
            if (!node) return;

            // Draw a small black dot for the node
            sf::CircleShape nodeDot(setup.dim / 4);
            nodeDot.setPosition(node->getPoint().getX(), node->getPoint().getY());
            nodeDot.setFillColor(nodeColor);
            window.draw(nodeDot);

            // Draw a line from the parent to the current node (if not root)
            if (node->getParent()) {
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(node->getParent()->getPoint().getX(), node->getParent()->getPoint().getY()), lineColor),
                    sf::Vertex(sf::Vector2f(node->getPoint().getX(), node->getPoint().getY()), lineColor)
                };
                window.draw(line, 2, sf::Lines);
            }

            // Recursively draw all the children of the current node
            for (const auto& child : node->getChildren()) {
                drawTree(child.get());
            }
        };

        // Start drawing the tree from the root node
        drawTree(root);

        // Draw the start point as a red dot
        sf::CircleShape startDot(setup.dim / 2);
        startDot.setPosition(setup.start.getX(), setup.start.getY());
        startDot.setFillColor(startEndColor);
        window.draw(startDot);

        // Draw the target point as a red dot
        sf::CircleShape targetDot(setup.dim / 2);
        targetDot.setPosition(setup.target.getX(), setup.target.getY());
        targetDot.setFillColor(startEndColor);
        window.draw(targetDot);

        // Draw the shortest path as a thick black line
        if (!path.empty()) {
            for (size_t i = 1; i < path.size(); ++i) {
                // Calculate the direction and length of the segment
                sf::Vector2f start(path[i - 1].getX(), path[i - 1].getY());
                sf::Vector2f end(path[i].getX(), path[i].getY());
                sf::Vector2f direction = end - start;

                // Calculate the angle of rotation
                float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
                float angle = std::atan2(direction.y, direction.x) * 180 / 3.14159f;

                // Create a rectangle to represent the thick line
                sf::RectangleShape pathSegment(sf::Vector2f(length, pathThickness));
                pathSegment.setPosition(start);
                pathSegment.setRotation(angle);
                pathSegment.setFillColor(pathColor);

                window.draw(pathSegment);
            }
        }

        window.display();  // Display everything on the screen
    }
}

std::string generateLogFileName() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << "logs/rrt_project_" << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S") << ".log";
    return oss.str();
}

void setupLogger() {
    std::string logFileName = generateLogFileName();
    logger = spdlog::basic_logger_mt("file_logger", logFileName);
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger); // Set as default to easily use spdlog::info, etc.

    // Test logging
    logger->info("Logger initialized and ready to log.");
}


// Templated function to add obstacles in a warehouse environment
template <typename T>
void addWarehouseObstacles(Setup<T>& setup, int width, int height) {
    int shelfWidth = width / 5;        // Divide the width into 5 equal parts
    int shelfHeight = height / 10;     // Divide the height into 10 equal parts
    int spaceBetweenShelves = height / 10; // Space between each shelf

    // List of obstacles, each obstacle defined by 4 corner points
    std::vector<std::vector<Point<T>>> obstacles;

    // Add obstacles in the first column
    for (int i = 0; i < 4; i++) {
        int x1 = shelfWidth; // x-coordinate for the shelf in the first column
        int y1 = (i * (shelfHeight + spaceBetweenShelves)) + spaceBetweenShelves;  // Top left Y
        int y2 = y1 + shelfHeight;  // Bottom Y

        // Add top-left, top-right, bottom-right, and bottom-left points
        std::vector<Point<T>> shelf = {
            Point<T>(x1, y1),                  // Top-left
            Point<T>(x1 + shelfWidth, y1),     // Top-right
            Point<T>(x1 + shelfWidth, y2),     // Bottom-right
            Point<T>(x1, y2)                   // Bottom-left
        };
        obstacles.push_back(shelf);  // Add to obstacles list
    }

    // Add obstacles in the second column
    for (int i = 0; i < 4; i++) {
        int x1 = width - 2 * shelfWidth; // x-coordinate for the shelf in the second column
        int y1 = (i * (shelfHeight + spaceBetweenShelves)) + spaceBetweenShelves;  // Top left Y
        int y2 = y1 + shelfHeight;  // Bottom Y

        // Add top-left, top-right, bottom-right, and bottom-left points
        std::vector<Point<T>> shelf = {
            Point<T>(x1, y1),                  // Top-left
            Point<T>(x1 + shelfWidth, y1),     // Top-right
            Point<T>(x1 + shelfWidth, y2),     // Bottom-right
            Point<T>(x1, y2)                   // Bottom-left
        };
        obstacles.push_back(shelf);  // Add to obstacles list
    }

    // Add each obstacle to the setup
    for (const auto& obstacle : obstacles) {
        setup.addObstacle(obstacle);
    }
}