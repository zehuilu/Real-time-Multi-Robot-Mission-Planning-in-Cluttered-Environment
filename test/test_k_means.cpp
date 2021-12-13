#include <iostream>
#include <vector>
#include <chrono>
#include <tuple>
#include "k_means.hpp"


int main()
{
    // {x0,y0, x1,y1, x2,y2, ...}
    std::vector<float> data = {100.0,100.0, 99.0,99.0, 100.0,99.0, 99.0,100.0,
                               0.0,0.0, 0.0,0.5, 0.5,0.0, 0.5,0.5,
                               -100.0,-100.0, -101.0,-101.0, -101.5,-101.5};

    size_t k = 3;
    size_t number_of_iterations = 1000;

    auto start = std::chrono::high_resolution_clock::now();

    auto [means, assignments, points_idx_for_clusters] = k_means(data, k, number_of_iterations);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time used [microseconds]:" << duration.count() << std::endl;

    std::cout << "Cluster centroid:" << std::endl;
    for (size_t idx = 0; idx < means.size()/2; ++idx) {
        std::cout << means[2*idx] << ", " << means[2*idx+1] << std::endl;
    }

    std::cout << "Each point belongs to which cluster:" << std::endl;
    for (auto& pt : assignments) {
        std::cout << pt << std::endl;
    }
    
    for (auto& vec : points_idx_for_clusters) {
        std::cout << "This is a cluster." << std::endl;
        for (auto& ele : vec) {
            std::cout << ele << std::endl;
        }
    }

    return 0;
}

