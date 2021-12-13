// Reference:
// https://en.wikipedia.org/wiki/K-means%2B%2B
// https://www.geeksforgeeks.org/ml-k-means-algorithm/
// https://www.kdnuggets.com/2020/06/centroid-initialization-k-means-clustering.html

#ifndef K_MEANS_PLUS_PLUS_HPP
#define K_MEANS_PLUS_PLUS_HPP

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>
#include <tuple>


// template <typename T> 
inline std::vector<float> k_means_plus_plus(
    const std::vector<float>& data,
    const size_t& k)
{
    static std::random_device seed;
    static std::mt19937 random_number_generator(seed());
    size_t num_data = data.size()/2;
    std::uniform_int_distribution<size_t> indices(0, num_data - 1);

    // initialize the first centroid randomly from the dataset.
    std::vector<float> centroids(2*k);
    size_t idx_first = indices(random_number_generator);
    centroids[0] = data[2*idx_first];
    centroids[1] = data[2*idx_first+1];

    for (size_t cluster = 1; cluster < k; ++cluster) {
        // initialize a list to store distances of data points from nearest centroid
        std::vector<float> dist_vec;

        for (size_t idx_data = 0; idx_data < num_data; ++idx_data) {
            auto best_distance = std::numeric_limits<float>::max();
            // compute distance of 'point' from each of the previously selected centroid and store the minimum distance
            for (size_t idx = 0; idx < centroids.size()/2; ++idx) {
                float distance = ((data[2*idx_data]-centroids[2*idx]) * (data[2*idx_data]-centroids[2*idx]) + 
                                 (data[2*idx_data+1]-centroids[2*idx+1]) * (data[2*idx_data+1]-centroids[2*idx+1]));

                if (distance < best_distance) best_distance = distance;
            }
            dist_vec.push_back(best_distance);
        }

        // select data point with maximum distance as our next centroid
        size_t idx_max = std::max_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
        centroids[2*cluster] = data[2*idx_max];
        centroids[2*cluster+1] = data[2*idx_max+1];
    }
    return centroids;
}


#endif