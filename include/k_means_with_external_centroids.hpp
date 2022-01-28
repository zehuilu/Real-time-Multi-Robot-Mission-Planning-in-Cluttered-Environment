#ifndef K_MEANS_WITH_EXTERNAL_CENTROIDS_HPP
#define K_MEANS_WITH_EXTERNAL_CENTROIDS_HPP

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>
#include <tuple>
#include <cmath>


inline std::tuple< std::vector<float>, std::vector<size_t>, std::vector<std::vector<size_t>>, std::vector<float> > k_means_with_external_centroids(
    const std::vector<float>& data,
    const size_t& k,
    const size_t& number_of_iterations,
    const std::vector<float>& means_previous)
{
    size_t num_data = data.size()/2;
    // initialize all centroids from previous centroids
    std::vector<float> means = means_previous;

    std::vector<size_t> assignments(num_data);
    for (size_t iteration = 0; iteration < number_of_iterations; ++iteration) {
        // find assignments.
        for (size_t idx_data = 0; idx_data < num_data; ++idx_data) {
            auto best_distance = std::numeric_limits<float>::max();
            size_t best_cluster = 0;
            for (size_t cluster = 0; cluster < k; ++cluster) {
                float distance = std::pow(data[2*idx_data]-means[2*cluster], 2) + 
                                 std::pow(data[2*idx_data+1]-means[2*cluster+1], 2);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_cluster = cluster;
                }
            }
            assignments[idx_data] = best_cluster;
        }

        // sum up and count points for each cluster.
        std::vector<float> points_sum(2*k);
        std::vector<size_t> counts(k, 0);
        for (size_t idx_data = 0; idx_data < num_data; ++idx_data) {
            auto cluster = assignments[idx_data];
            points_sum[2*cluster] += data[2*idx_data];
            points_sum[2*cluster+1] += data[2*idx_data+1];
            counts[cluster] += 1;
        }

        // divide sums by counts to get new centroids.
        for (size_t cluster = 0; cluster < k; ++cluster) {
            // if cluster exists, compute it
            if (counts[cluster] > 0) {
                means[2*cluster] = points_sum[2*cluster] / counts[cluster];
                means[2*cluster+1] = points_sum[2*cluster+1] / counts[cluster];
            }
            // else, the cluster is away from the map
            else {
                means[2*cluster] = -1E5;
                means[2*cluster+1] = -1E5;
            }
        }
    }

    // create a 2D vector which contains points in different clusters
    std::vector<size_t> points_index_1d;
    std::vector<std::vector<size_t>> points_idx_for_clusters(k, points_index_1d);
    for (size_t idx_data = 0; idx_data < num_data; ++idx_data) {
        points_idx_for_clusters[assignments[idx_data]].push_back(idx_data);
    }

    // create a 1D vector, each element is the summation of distance between a cluster centroid and its associated points
    std::vector<float> sum_distance_vec(k, 0.0);
    for (size_t cluster = 0; cluster < k; ++cluster) {
        for (size_t idx = 0; idx < points_idx_for_clusters[cluster].size(); ++idx) {
            size_t pt_idx = points_idx_for_clusters[cluster][idx];
            float this_distance = std::sqrt(
                std::pow(data[2*pt_idx]-means[2*cluster], 2) +
                std::pow(data[2*pt_idx+1]-means[2*cluster+1], 2));
            sum_distance_vec[cluster] = sum_distance_vec[cluster] + this_distance;
        }
    }

    return {means, assignments, points_idx_for_clusters, sum_distance_vec};
}


#endif