#include "arena_definitions.cpp"

// Main function
int main() {
    // Set up logger
    setupLogger();

    // Parameters
    int width = 1000, height = 1000, step_size = 50;
    auto start_time = std::chrono::high_resolution_clock::now();

    // RRT Setup
    Setup<int> setup(10, 10, width, height, Point<int>(10, 10), Point<int>(950, 950), step_size);

    // Add warehouse obstacles
    addWarehouseObstacles(setup, width, height);

    // Prometheus setup remains in main
    prometheus::Exposer exposer{"127.0.0.1:8080"};
    auto registry = std::make_shared<prometheus::Registry>();
    auto& gauge_family = prometheus::BuildGauge()
                            .Name("rrt_execution_time")
                            .Help("RRT execution time in milliseconds")
                            .Register(*registry);
    auto& gauge = gauge_family.Add({{"metric", "execution_time"}});
    
    // Start RRT
    RRTPlanner<int> rrt(setup);
    rrt.start(1);
    std::vector<Point<int>> path = rrt.getShortestPath();

    // End timer after target is reached
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // Log the duration
    auto logger = spdlog::get("file_logger");
    logger->info("RRT completed in {} milliseconds.", duration);

    // Set the execution time to the Prometheus gauge
    gauge.Set(static_cast<double>(duration));

    // Visualize the RRT and obstacles
    visualize(setup, rrt.getRoot().get(), path);

    return 0;
}
