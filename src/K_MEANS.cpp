#include <vector>
#include <tuple>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "k_means.hpp"
#include "k_means_with_plus_plus.hpp"


inline std::tuple< std::vector<float>, std::vector<size_t>, std::vector<std::vector<size_t>> > KMeans(
    const std::vector<float>& data,
    const size_t& k,
    const size_t& number_of_iterations)
{
    return k_means(data, k, number_of_iterations);
}


inline std::tuple< std::vector<float>, std::vector<size_t>, std::vector<std::vector<size_t>>, std::vector<float> > KMeansWPP(
    const std::vector<float>& data,
    const size_t& k,
    const size_t& number_of_iterations)
{
    return k_means_with_plus_plus(data, k, number_of_iterations);
}


inline PYBIND11_MODULE(K_MEANS, module) {
    module.doc() = "Python wrapper of K-means clustering";

    module.def("KMeans", &KMeans, "K-means clustering");

    module.def("KMeansWPP", &KMeansWPP, "K-means clustering with K-means++ initialization");
}
