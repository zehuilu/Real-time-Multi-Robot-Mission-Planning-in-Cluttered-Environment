#!/usr/bin/env python3
import time
import numpy as np
import matplotlib.pyplot as plt
import sklearn.datasets
import sklearn.cluster
import scipy.cluster.vq
import pathmagic
with pathmagic.context():
    import K_MEANS

num_data = 50
num_cluster = 3
number_of_iterations = 300

# Generate fake data
# data is a 2D numpy array, [[x0,y0], [x1,y1], [x2,y2], ...]
data, labels = sklearn.datasets.make_blobs(
    n_samples=num_data, n_features=2, centers=num_cluster)

# run K-means c++ library with random initialization
t0 = time.time()
means_cpp_random, assignments, points_idx_for_clusters = K_MEANS.KMeans(data.flatten(), num_cluster, number_of_iterations)
t1 = time.time()
means_cpp_random = np.array(means_cpp_random).reshape(-1, 2)
print("cpp time with random initialization [sec]: " + str(t1-t0))
print("means_cpp_random")
print(means_cpp_random)

# run K-means c++ library with K-means++
t0 = time.time()
means_cpp_1d, assignments, points_idx_for_clusters, sum_distance_vec = K_MEANS.KMeansPP(data.flatten(), num_cluster, number_of_iterations)
t1 = time.time()
means_cpp = np.array(means_cpp_1d).reshape(-1, 2)
print("cpp time with K-means++ [sec]: " + str(t1-t0))
print("means_cpp")
print(means_cpp)

# run K-means c++ library with external centroids
t0 = time.time()
means_cpp_ec, assignments_ec, points_idx_for_clusters_ec, sum_distance_vec_ec = K_MEANS.KMeansEC(data.flatten(), num_cluster, number_of_iterations, means_cpp_1d)
t1 = time.time()
means_cpp_ec = np.array(means_cpp_ec).reshape(-1, 2)
print("cpp time with external centroids [sec]: " + str(t1-t0))
print("means_cpp_ec")
print(means_cpp_ec)

# scipy
t0 = time.time()
means_scipy, _ = scipy.cluster.vq.kmeans(data, num_cluster, iter=number_of_iterations)
t1 = time.time()
print("scipy time [sec]: " + str(t1-t0))
print("means_scipy")
print(means_scipy)

# scikit-learn
t0 = time.time()
kmeans_scikit = sklearn.cluster.KMeans(num_cluster, max_iter=number_of_iterations)
kmeans_scikit.fit(data)
means_scikit = kmeans_scikit.cluster_centers_
t1 = time.time()
print("scikit-learn time [sec]: " + str(t1-t0))

# visualization
# K-means c++ with random initialization
plt.figure(1)
plt.scatter(data[:, 0], data[:, 1], c=labels)
plt.scatter(means_cpp_random[:, 0], means_cpp_random[:, 1], linewidths=2, color='red')
plt.title("c++ with random initialization")

# K-means c++ with K-means++
plt.figure(2)
plt.scatter(data[:, 0], data[:, 1], c=labels)
plt.scatter(means_cpp[:, 0], means_cpp[:, 1], linewidths=2, color='red')
plt.title("c++ with K-means++")

print("sum_distance_vec")
print(sum_distance_vec)

# K-means c++ with external centroids
plt.figure(3)
plt.scatter(data[:, 0], data[:, 1], c=labels)
plt.scatter(means_cpp_ec[:, 0], means_cpp_ec[:, 1], linewidths=2, color='red')
plt.title("c++ with external centroids")

# scipy
plt.figure(4)
plt.scatter(data[:, 0], data[:, 1], c=labels)
plt.scatter(means_scipy[:, 0], means_scipy[:, 1], linewidths=2, color='red')
plt.title("scipy")

# scikit-learn
plt.figure(5)
plt.scatter(data[:, 0], data[:, 1], c=labels)
plt.scatter(means_scikit[:, 0], means_scikit[:, 1], linewidths=2, color='red')
plt.title("scikit-learn")

plt.show()
