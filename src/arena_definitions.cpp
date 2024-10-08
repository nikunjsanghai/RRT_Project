#include"arena_setup.h"

template <typename T>
void Setup<T>::addObstacle(const std::vector<Point<T>>& obstacle) {
    if (obstacle.size() != 4) {
        logger->error("Error: Obstacle must be defined by 4 corners.");
        return;
    }

    // Assume the obstacle is defined by 4 corners: top-left, top-right, bottom-right, bottom-left
    Point<T> topLeft = obstacle[0];
    Point<T> topRight = obstacle[1];
    Point<T> bottomRight = obstacle[2];
    Point<T> bottomLeft = obstacle[3];

    // Get the min and max X and Y values from the corners
    T minX = std::min({topLeft.getX(), topRight.getX(), bottomRight.getX(), bottomLeft.getX()});
    T maxX = std::max({topLeft.getX(), topRight.getX(), bottomRight.getX(), bottomLeft.getX()});
    T minY = std::min({topLeft.getY(), topRight.getY(), bottomRight.getY(), bottomLeft.getY()});
    T maxY = std::max({topLeft.getY(), topRight.getY(), bottomRight.getY(), bottomLeft.getY()});

    // Iterate over all points in the rectangular region and mark them as obstacles
    for (T x = minX; x <= maxX; x += dim) {
        for (T y = minY; y <= maxY; y += dim) {
            Point<T> point(x, y);
            markCell(point, -1); // Mark the cell as an obstacle (-1)
        }
    }
    logger->info("Obstacle added with top-left: ({}, {}), bottom-right: ({}, {})", minX, minY, maxX, maxY);
}
template <typename T>
Node<T>* RRTPlanner<T>::findNearest(const Point<T>& randomPoint, Node<T>* currentNode, double& minDistance) {
        Node<T>* nearestNode = nullptr;
        std::queue<Node<T>*> queue;
        queue.push(currentNode);

        while (!queue.empty()) {
            Node<T>* node = queue.front();
            queue.pop();

            double distance = calculateDistance(node->getPoint(), randomPoint);
            if (distance < minDistance) {
                minDistance = distance;
                nearestNode = node;
            }

            for (const auto& child : node->getChildren()) {
                queue.push(child.get());
            }
        }
        return nearestNode;
    }
template <typename T>
Point<T> RRTPlanner<T>::samplePoint() {
    Point<T> point = Point<T>(distX(gen), distY(gen));
    logger->debug("Sampled Point: ({}, {})", point.getX(), point.getY());
    return point;
}


template <typename T>
  void RRTPlanner<T>::addNode(Node<T>* nearestNode, const Point<T>& newPoint) {
        std::unique_lock<std::mutex> lock(treeMutex);
        nearestNode->addChild(std::make_unique<Node<T>>(newPoint, nearestNode));
        setup.markCell(newPoint, 1);
    }
template <typename T>
bool RRTPlanner<T>::collision_avoidance_check(Point<T>& randomPoint, const Point<T>& nearestPoint) {
    double dx = randomPoint.getX() - nearestPoint.getX();
    double dy = randomPoint.getY() - nearestPoint.getY();
    double distance = std::sqrt(dx * dx + dy * dy);

    logger->debug("Checking collision from ({}, {}) to ({}, {})", nearestPoint.getX(), nearestPoint.getY(), randomPoint.getX(), randomPoint.getY());

    if (distance == 0) {
        return false;
    }

    // Calculate the step ratio to limit the distance to step_size
    double step_ratio = std::min(setup.step_size / distance, 1.0);

    // Modify the random point to reflect the maximum step_size distance
    randomPoint.modify_x(nearestPoint.getX() + step_ratio * dx);
    randomPoint.modify_y(nearestPoint.getY() + step_ratio * dy);

    logger->debug("Modified random point to: ({}, {})", randomPoint.getX(), randomPoint.getY());

    // Now check the path from nearestPoint to randomPoint in steps of 'dim'
    int steps = static_cast<int>(distance / setup.dim);
    double step_dx = dx / steps;
    double step_dy = dy / steps;

    for (int i = 1; i <= steps; i++) {
        double intermediate_x = nearestPoint.getX() + i * step_dx;
        double intermediate_y = nearestPoint.getY() + i * step_dy;
        if(abs(intermediate_x -nearestPoint.getX()) < setup.dim && abs(intermediate_y -nearestPoint.getY()) < setup.dim)
        {
            continue;
        }
        if (!setup.isValid(Point<T>(intermediate_x, intermediate_y))) {
            logger->warn("Path blocked at: ({}, {})", intermediate_x, intermediate_y);
            return false; // Obstacle detected
        }
    }

    return true; // Path is clear
}


template <typename T>
void RRTPlanner<T>::run(int thread_id) {
    logger->debug("Thread {} started running.", thread_id);


    while (!targetReached) {
        Point<T> randomPoint = samplePoint();
        double minDistance = std::numeric_limits<double>::max();

        // Find the nearest point in the tree
        std::unique_lock<std::mutex> lock(treeMutex);
        Node<T>* nearestNode = findNearest(randomPoint, root.get(), minDistance);
        lock.unlock();
        if (nearestNode != nullptr) 
        {
        logger->debug("Thread {}: Nearest Node found at ({}, {})", thread_id, nearestNode->getPoint().getX(), nearestNode->getPoint().getY());
        } 
        else 
        {
         logger->debug("Thread {}: Nearest Node is nullptr.", thread_id);
        }
        if (nearestNode && abs(nearestNode->getPoint().getX() - randomPoint.getX()) > setup.dim && abs(nearestNode->getPoint().getY() - randomPoint.getY()) > setup.dim) {  
            // Adjust randomPoint to a point within step_size distance and check if the path is clear
            if (collision_avoidance_check(randomPoint, nearestNode->getPoint())) {
                logger->debug("Thread {}: Path is clear.", thread_id);
                std::unique_lock<std::mutex> lock(treeMutex);
                nearestNode->addChild(std::make_unique<Node<T>>(randomPoint, nearestNode));
                setup.markCell(randomPoint, 1);
                logger->info("Thread {}: Added point {} at ({}, {})", thread_id, count++, randomPoint.getX(), randomPoint.getY());
                lock.unlock();

                // Check if the target has been reached
                if (calculateDistance(randomPoint, setup.target) < setup.dim*1.5) {
                    logger->info("Thread {}: Target reached!", thread_id);
                    std::unique_lock<std::mutex> lock(treeMutex);
                    targetReached = true;
                    cv.notify_all();
                    logger->info("Thread {}: Notified all threads", thread_id);
                    lock.unlock();
                }
            }
        }
        else if(nearestNode)
        {
            logger->debug("Thread {}: Random point too close to nearest node.", thread_id);
        }
        else
        {
            logger->warn("Thread {}: Nearest node is nullptr.", thread_id);
        }
    }
}



template <typename T>
void RRTPlanner<T>::start(int num_threads) {
    std::vector<std::thread> threads;

    // Launch multiple threads
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back(&RRTPlanner::run, this, i);
    }

    std::unique_lock<std::mutex> lock(treeMutex);
    cv.wait(lock, [this] { return targetReached; });
    lock.unlock();

    // Join all threads once the target is reached
    for (auto& thread : threads) {
        thread.join();
    }
}

template <typename T>
std::vector<Point<T>> RRTPlanner<T>::getShortestPath() {
    std::queue<Node<T>*> nodeQueue;
    std::unordered_map<Node<T>*, Node<T>*> parentMap; // To track the parent of each node
    std::vector<Point<T>> path;
    
    // Start BFS from the root node
    nodeQueue.push(root.get());
    parentMap[root.get()] = nullptr;

    Node<T>* targetNode = nullptr;

    // Perform BFS to find the target node
    while (!nodeQueue.empty()) {
        Node<T>* currentNode = nodeQueue.front();
        nodeQueue.pop();

        // Check if we've reached the target node
        if (calculateDistance(currentNode->getPoint(), setup.target) < setup.dim * 1.5) {
            targetNode = currentNode;
            break;
        }

        // Add children to the queue
        for (const auto& child : currentNode->getChildren()) {
            nodeQueue.push(child.get());
            parentMap[child.get()] = currentNode; // Map the child to its parent
        }
    }

    // If targetNode is found, trace the path from the target node back to the root
    if (targetNode) {
        Node<T>* currentNode = targetNode;
        while (currentNode != nullptr) {
            path.push_back(currentNode->getPoint());
            currentNode = parentMap[currentNode]; // Move to the parent node
        }

        // Reverse the path to start from the root
        std::reverse(path.begin(), path.end());
    } else {
        logger->error("Target node not reachable.");
    }

    return path;
}

