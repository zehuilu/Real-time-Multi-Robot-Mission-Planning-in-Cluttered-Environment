// Adapted from Peter Goldsborough's website.
// http://www.goldsborough.me/c++/python/cuda/2017/09/10/20-32-46-exploring_k-means_in_python,_c++_and_cuda/

#ifndef K_MEANS_WITH_PLUS_PLUS_HPP
#define K_MEANS_WITH_PLUS_PLUS_HPP

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>
#include <tuple>
#include "k_means_plus_plus.hpp"


// template <typename T> 
inline std::tuple< std::vector<float>, std::vector<size_t>, std::vector<std::vector<size_t>> > k_means_with_plus_plus(
    const std::vector<float>& data,
    const size_t& k,
    const size_t& number_of_iterations)
{
    size_t num_data = data.size()/2;
    // initialize all centroids by K-means++
    std::vector<float> means = k_means_plus_plus(data, k);

    std::vector<size_t> assignments(num_data);
    for (size_t iteration = 0; iteration < number_of_iterations; ++iteration) {
        // find assignments.
        for (size_t idx_data = 0; idx_data < num_data; ++idx_data) {
            auto best_distance = std::numeric_limits<float>::max();
            size_t best_cluster = 0;
            for (size_t cluster = 0; cluster < k; ++cluster) {
                float distance = ((data[2*idx_data]-means[2*cluster]) * (data[2*idx_data]-means[2*cluster]) + 
                                 (data[2*idx_data+1]-means[2*cluster+1]) * (data[2*idx_data+1]-means[2*cluster+1]));
                    
                if (distance < best_distance) {
                    best_distance = distance;
                    best_cluster = cluster;
                }
            }
            assignments[idx_data] = best_cluster;
        }

        // sum up and count points for each cluster.
        std::vector<float> new_means(2*k);
        std::vector<size_t> counts(k, 0);
        for (size_t idx_data = 0; idx_data < num_data; ++idx_data) {
            auto cluster = assignments[idx_data];
            new_means[2*cluster] += data[2*idx_data];
            new_means[2*cluster+1] += data[2*idx_data+1];
            counts[cluster] += 1;
        }

        // divide sums by counts to get new centroids.
        for (size_t cluster = 0; cluster < k; ++cluster) {
            // turn 0/0 into 0/1 to avoid zero division.
            auto count = std::max<size_t>(1, counts[cluster]);
            means[2*cluster] = new_means[2*cluster] / count;
            means[2*cluster+1] = new_means[2*cluster+1] / count;
        }
    }

    // create a 2D vector which contains points in different clusters
    std::vector<size_t> points_index_1d;
    std::vector<std::vector<size_t>> points_idx_for_clusters(k, points_index_1d);
    for (size_t idx_data = 0; idx_data < num_data; ++idx_data) {
        points_idx_for_clusters[assignments[idx_data]].push_back(idx_data);
    }

    return {means, assignments, points_idx_for_clusters};
}


#endif