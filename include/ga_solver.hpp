#ifndef GA_SOLVER_HPP
#define GA_SOLVER_HPP


#include <limits>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <tuple>
#include <typeinfo>


static constexpr float LARGE_NUM = 1E8;

std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
std::uniform_real_distribution<float> dist_01(0.0, 1.0);  // uniform real distribution [0.0, 1.0]


// Structure of a GNOME 
// vector defined the task execution order, and the fitness value is the cost of executing this order
struct individual {
    std::vector<int> gnome;
    float fitness;
};

// generate a random integer number in [min, max]
inline int rand_integer(const int& min, const int& max)
{
    std::uniform_int_distribution<int> uni(min, max); // guaranteed unbiased
    return uni(rng);
}

// generate two unrepeated random integers without the while loop
inline void rand_integer_two(const std::vector<int>& idx_mutate_vector, int* rand_num_array)
{
    std::vector<int> idx_mutate_vector_copy(idx_mutate_vector);

    // generate the first random number
    rand_num_array[0] = rand_integer(1, idx_mutate_vector.size());
    // erase the first random number.
    // if the random mumber is 4, the index of the original vector is 4-1=3.
    idx_mutate_vector_copy.erase(idx_mutate_vector_copy.begin() + rand_num_array[0] - 1);

    // generate a random index for the new vector
    //int index_2 = rand_integer(0, idx_mutate_vector_copy.size() - 1);

    rand_num_array[1] = idx_mutate_vector_copy[rand_integer(0, idx_mutate_vector_copy.size()-1)];

    // return rand_num_array_ptr
}

// Function to return a mutated GNOME 
// Mutated GNOME is a std::vector<int> with a random interchange of two genes to create variation in species
inline std::vector<int> mutatedGene(const std::vector<int>& gnome, const std::vector<int>& idx_mutate_vector)
{
    std::vector<int> gnome_new(gnome);
    int rand_num_array[2];
    rand_integer_two(idx_mutate_vector, rand_num_array);
    std::iter_swap(gnome_new.begin() + rand_num_array[0], gnome_new.begin() + rand_num_array[1]);

    return gnome_new;
}

// Function to return a valid GNOME vector 
// required to create the population 
inline std::vector<int> create_gnome(const std::vector<int>& idx_mutate_vector)
{
    std::vector<int> gnome(idx_mutate_vector.size()+1, 0);
    std::vector<int> idx_mutate_vector_copy(idx_mutate_vector);

    for (size_t i = 1; i <= idx_mutate_vector.size(); ++i) {
        // generate a random index
        int idx_now = rand_integer(0, idx_mutate_vector_copy.size() - 1);
        // slice the vector
        gnome[i] = idx_mutate_vector_copy[idx_now];
        // erase the used one
        idx_mutate_vector_copy.erase(idx_mutate_vector_copy.begin() + idx_now);
    }

    return gnome;
}

// Function to return the fitness value of a gnome. 
// The fitness value is the path length of the path represented by the GNOME. 
inline float compute_fitness(const std::vector<int>& gnome, const std::vector<std::vector<float>>& distance_matrix)
{
    float f = 0;
    for (size_t i = 0; i < gnome.size() - 1; i++) {
        if (distance_matrix[gnome[i]][gnome[i + 1]] >= LARGE_NUM)
            return LARGE_NUM;
        f += distance_matrix[gnome[i]][gnome[i + 1]];
    }
    return f;
}

// Function to return the updated value of the cooling element. 
inline float cooldown(const float& temperature, const size_t& iter, const size_t& max_iter, const float& temperature_init)
{
    // when the iteration is about 70% of max_iter, the temperature is designed to be less than 0.1 * temperature_init.
    // then, decrease the temperature quickly such that there is more possibility to reject the bad gnome.
    if (iter < 0.7 * max_iter) return temperature - (0.9 * temperature_init) / (0.7 * max_iter);
    else return 0.9 * temperature;
}

// Comparator for GNOME struct. 
inline bool lessthan(const struct individual& t1, const struct individual& t2)
{
    return t1.fitness < t2.fitness;
}

// Utility function for TSP problem. 
inline std::tuple< std::vector<int>, float > Run(
    const size_t& num_nodes,
    const size_t& pop_size,
    const size_t& max_iter,
    const std::vector<std::vector<float>>& distance_matrix,
    const bool& print_flag)
{
    // Generation Number 
    size_t iter = 0;

    /*
    !! Suppose we have 5(num_nodes) cities, the chromosome has 5 numbers, because we don't want to come back.
    But the first element won't mutate, so the mutateable numbers are num_nodes-1 = 4.
    In this case, the vector is [1, 2, 3, 4]
    */
    std::vector<int> idx_mutate_vector(num_nodes - 1);
    std::iota(idx_mutate_vector.begin(), idx_mutate_vector.end(), 1);

    std::vector<struct individual> population(pop_size);
    struct individual indiv_temp;

    // Populating the GNOME pool. 
    for (size_t i = 0; i < pop_size; i++) {
        indiv_temp.gnome = create_gnome(idx_mutate_vector);
        indiv_temp.fitness = compute_fitness(indiv_temp.gnome, distance_matrix);
        population[i] = indiv_temp;
    }

    sort(population.begin(), population.end(), lessthan);

    if (print_flag) {
        std::cout << "\nInitial population: " << std::endl << "GNOME     FITNESS VALUE\n";
        for (size_t i = 0; i < pop_size; ++i) {
            for (auto ele : population[i].gnome) std::cout << ele << "->";
            std::cout << "   " << population[i].fitness << std::endl;
        }
        std::cout << "\n";
    }

    float temperature_init = 100;
    float temperature = temperature_init;

    // Iteration to perform 
    // population crossing and gene mutation. 
    while (temperature > 0.1 && iter < max_iter) {
        if (print_flag) std::cout << "\nCurrent temp: " << temperature << "\n";

        std::vector<struct individual> new_population(pop_size);

        for (size_t i = 0; i < pop_size; i++) {

            while (true) {
                std::vector<int> new_g = mutatedGene(population[i].gnome, idx_mutate_vector);


                struct individual new_gnome;
                new_gnome.gnome = new_g;
                new_gnome.fitness = compute_fitness(new_gnome.gnome, distance_matrix);

                if (new_gnome.fitness <= population[i].fitness) {
                    new_population[i] = new_gnome;
                    break;
                }
                else {
                    float random_prob = dist_01(rng);

                    // Accepting the rejected children at 
                    // a possible probablity above threshold. 
                    float prob = pow(2.71828, -1*(new_gnome.fitness-population[i].fitness)/temperature);
                    if (random_prob < prob) {
                        new_population[i] = new_gnome;
                        break;
                    }
                }
            }
        }

        // cooling down
        temperature = cooldown(temperature, iter, max_iter, temperature_init);

        population = new_population;
        sort(population.begin(), population.end(), lessthan);

        if (print_flag) {
            std::cout << "Generation " << iter << " \n";
            std::cout << "GNOME     FITNESS VALUE\n";
            for (size_t i = 0; i < pop_size; ++i) {
                for (auto ele : population[i].gnome) std::cout << ele << "->";
                std::cout << "   " << population[i].fitness << std::endl;
            }
        }

        iter++;
    }

    return {population[0].gnome, population[0].fitness};
}

#endif