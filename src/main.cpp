#include"arena_definitions.cpp"




// Main function
int main() {
    int width = 1000, height = 1000, step_size = 50;
    Setup<int> setup(10,10,width, height, Point<int>(10, 10), Point<int>(850, 850), step_size);

    std::vector<Point<int>> obstacle = {Point<int>(400, 400), Point<int>(400, 700), Point<int>(800, 700), Point<int>(800, 400)};
    setup.addObstacle(obstacle);

    RRTPlanner<int> rrt(setup);
    rrt.start(10);

    return 0;
}
