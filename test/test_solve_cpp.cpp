#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include "../externals/Lazy-Theta-with-optimization-any-angle-pathfinding/include/tileadaptor.hpp"
#include "../include/solve_test.hpp"


int main()
{
    constexpr int mapSizeX = 70; // width
    constexpr int mapSizeY = 20; // length

    // initialize the map, 0 means no obstacles; each sub vector is a column.
    std::vector<std::vector<int>> Map(mapSizeX, std::vector<int> (mapSizeY, 0));

    // Define the position for the agent
    int agent_position[2] = {1, 1};
    // Define a set of targets positions [x0,y0, x1,y1, x2,y2]
    std::vector<int> targets_position{mapSizeX-2,mapSizeY-2, 15,15, 50,15};

    // This is a lambda function to create the wall
    auto makeWall = [&Map](const Vectori& pos, const Vectori& size)
    {
        for(int x = 0; x < size.x; x++)
        {
            for(int y = 0; y < size.y; y++)
            {
                Map[pos.x + x][pos.y + y] = 255;
            }
        }
    };

    //borders
    makeWall({0, 0}, {mapSizeX, 1});
    makeWall({0, 0}, {1, mapSizeY});
    makeWall({0, mapSizeY - 1}, {mapSizeX, 1});
    makeWall({mapSizeX - 1, 0}, {1, mapSizeY});

    //walls
    makeWall({5, 0}, {1, mapSizeY - 6});
    makeWall({mapSizeX - 6, 5}, {1, mapSizeY - 6});

    makeWall({mapSizeX - 6, 5}, {4, 1});
    makeWall({mapSizeX - 4, 8}, {4, 1});

    makeWall({20, 0}, {1, mapSizeY - 4});
    makeWall({mapSizeX - 20, 5}, {14, 1});

    //start and end point
    Map[agent_position[0]][agent_position[1]] = 0;
    Map[targets_position[0]][targets_position[1]] = 0;

    // a conversion from 2D (column major) map to a 1D (row major) map
    std::vector<int> Map_1D;
    Map_1D.reserve(mapSizeX * mapSizeY);
    for(int y = 0; y < mapSizeY; y++)
    {
        for(int x = 0; x < mapSizeX; x++)
        {
            Map_1D.push_back(Map[x][y]);
        }
    }

    auto start = std::chrono::high_resolution_clock::now();

    
    // solve it
    auto [path_many, distances_many, distance_matrix] = solve_test(agent_position, targets_position, Map_1D, mapSizeX, mapSizeY);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time used [microseconds]:" << duration.count() << std::endl;

    for(long unsigned int i=0; i<path_many.size(); i++)
    {
        std::cout << "This is a path, idx = " << i << std::endl;
        std::cout << "Total distance is: " << distances_many[i] << std::endl;
        if (path_many[i].size() > 0)
        {
            for (long unsigned int j=0; j<path_many[i].size(); j = j + 2)
            {
                std::cout << path_many[i][j] << "," << path_many[i][j+1] << std::endl;
            }
        }
        else
        {
            std::cout << "This is an empty path. No path!" << std::endl;
        }
        
    }


    return 0;
}
