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
#include <condition_variable>
#include <SFML/Graphics.hpp>

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
    void nearestNodeprint()
    {
        std::cout << "Nearest Node: ";
        point.print();
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
        std::cout << "Initializing Setup..." << std::endl;
        std::cout << "Length: " << length << ", Width: " << width << std::endl;
        std::cout << "Start Point: (" << start.getX() << ", " << start.getY() << ")" << std::endl;
        std::cout << "Target Point: (" << target.getX() << ", " << target.getY() << ")" << std::endl;
        std::cout << "Robot Length: " << l << ", Robot Width: " << w << std::endl;
        std::cout << "Step Size: " << step_size << std::endl;
        std::cout << "Arena Dimensions: " << arena.size() << "x" << arena[0].size() << std::endl;
        std::cout << "unit cell size: " << dim << std::endl;
        std::cout << "Setup complete." << std::endl;
    }

    bool isValid(const Point<T>& point) const {
        int x = static_cast<int>(point.getX() / dim);
        int y = static_cast<int>(point.getY() / dim);
        return x >= 0 && x < std::ceil(length / dim) && y >= 0 && y < std::ceil(width/ dim) && arena[y][x] == 0;
    }

    void markCell(const Point<T>& point, int value) {
    int x = static_cast<int>(point.getX() / dim);
    int y = static_cast<int>(point.getY() / dim);
    
    if (x >= 0 && x < arena[0].size() && y >= 0 && y < arena.size()) {
        arena[y][x] = value;
    } else {
        std::cerr << "Warning: Tried to mark cell out of bounds!" << std::endl;
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
        : gen(std::random_device{}()), distX(0, setup.length), distY(0, setup.width), count(1), setup(setup), root(std::make_unique<Node<T>>(setup.start)) {
        setup.markCell(setup.start, 1);
    }

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

    // Start the RRT planner with multiple threads
    void start(int num_threads);

private:  // If you have private members or helper functions, declare them here
    double calculateDistance(const Point<T>& p1, const Point<T>& p2) {
        return std::sqrt(std::pow(p1.getX() - p2.getX(), 2) + std::pow(p1.getY() - p2.getY(), 2));
    }
};

