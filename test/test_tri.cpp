#include <iostream>
#include <vector>
#include <math.h>


int main() {

    int num_targets = 4;
    std::vector<int> distance = {3,4,5,6,7,8,9,10,11,12};


    // unsigned long num_targets = 3;
    // std::vector<int> distance = {3,4,5,6,7,8};


    unsigned long n = num_targets + 1;
    std::cout << "This is the tri array: " << std::endl;
    for (unsigned long idx = 0; idx < distance.size(); idx++)
    {
        std::cout << distance[idx] << ", ";
    }
    std::cout << std::endl;

    std::vector<std::vector<int>> mat(num_targets+1, std::vector<int>(num_targets+1, 0));
    std::cout << "This is the matrix: " << std::endl;
    for (unsigned long i = 0; i < mat.size(); i++)
    {
        for (unsigned long j = 0; j < mat[i].size(); j++)
        {
            std::cout << mat[i][j] << ", ";
        }
        std::cout << std::endl;
    }


    std::vector<std::vector<int>> mat1 = mat;
    for (unsigned long idx = 0; idx < distance.size(); idx++)
    {
        unsigned long i = n - 2 - floor(sqrt(-8*idx + 4*n*(n-1)-7)/2.0 - 0.5);
        unsigned long j = idx + i + 1 - n*(n-1)/2 + (n-i)*((n-i)-1)/2;

        mat1[i][j] = distance[idx];
        mat1[j][i] = distance[idx];
    }
    std::cout << "This is mat1: " << std::endl;
    for (unsigned long i = 0; i < mat1.size(); i++)
    {
        for (unsigned long j = 0; j < mat1[i].size(); j++)
        {
            std::cout << mat1[i][j] << ", ";
        }
        std::cout << std::endl;
    }


    std::vector<std::vector<int>> mat2 = mat;
    for (unsigned long i = 0; i < n; i++)
    {
        for (unsigned long j = i + 1; j < n; j++)
        {
            int k = (n*(n-1))/2 - ((n-i)*((n-i)-1))/2 + j - i - 1;
            mat2[i][j] = distance[k];
        }
    }
    std::cout << "This is mat2: " << std::endl;
    for (unsigned long i = 0; i < mat2.size(); i++)
    {
        for (unsigned long j = 0; j < mat2[i].size(); j++)
        {
            std::cout << mat2[i][j] << ", ";
        }
        std::cout << std::endl;
    }

    return 0;
}