#include "arena_definitions.cpp"

int main() {
    // Set up logger - this initializes the global logger variable
    setupLogger();

    // Parameters for the simulation
    int width = 1000, height = 1000, step_size = 50;
    auto start_time = std::chrono::high_resolution_clock::now();

    // RRT Setup
    Setup<int> setup(10, 10, width, height, Point<int>(10, 10), Point<int>(950, 950), step_size);
    addWarehouseObstacles(setup, width, height); // Assume this function is defined elsewhere

    // Prometheus metric setup
    prometheus::Exposer exposer{"127.0.0.1:8080"};
    auto registry = std::make_shared<prometheus::Registry>();
    auto& gauge_family = prometheus::BuildGauge()
                            .Name("rrt_execution_time")
                            .Help("RRT execution time in milliseconds")
                            .Register(*registry);
    auto& gauge = gauge_family.Add({{"metric", "execution_time"}});

    // Start the RRT Planner
    RRTPlanner<int> rrt(setup);
    rrt.start(1);
    std::vector<Point<int>> path = rrt.getShortestPath();

    // Stop timing after target is reached
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    logger->info("RRT completed in {} milliseconds.", duration);

    // Record the duration in Prometheus
    gauge.Set(static_cast<double>(duration));

    // Visualize the RRT and obstacles
    visualize(setup, rrt.getRoot().get(), path);

    return 0;
}
