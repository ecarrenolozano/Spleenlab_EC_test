#include <iostream>
#include <thread>
#include <vector>
#include <cmath>

// Function to perform CPU-intensive computations
void cpuIntensiveTask() {
    while (true) {
        // Perform some intensive computations
        double result = 0.0;
        for (int i = 0; i < 1000000; ++i) {
            result += std::sin(i) * std::cos(i);
        }
    }
}

int main() {
    // Get the number of available CPU cores
    unsigned int numCores = std::thread::hardware_concurrency();

    // Create a vector to store threads
    std::vector<std::thread> threads;

    // Start a thread on each CPU core
    for (unsigned int i = 0; i < numCores - 1; ++i) {
        threads.push_back(std::thread(cpuIntensiveTask));
    }

    // Join all threads (this will never be reached since threads run indefinitely)
    for (auto& thread : threads) {
        thread.join();
    }

    return 0;
}