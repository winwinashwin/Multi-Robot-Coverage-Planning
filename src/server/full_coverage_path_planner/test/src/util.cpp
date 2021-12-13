//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 27-9-18.
//

#include <full_coverage_path_planner/util.h>
#include <vector>

std::vector<std::vector<bool>> makeTestGrid(int x, int y, bool fill)
{
    auto grid = std::vector<std::vector<bool>>(y, std::vector<bool>(x, fill));
    return grid;
}

bool operator==(const Point_t& lhs, const Point_t& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }

bool randomFillTestGrid(std::vector<std::vector<bool>>& grid, float obstacle_fraction)
{
    unsigned int seed = time(nullptr);
    auto max_y = grid.size();
    if (max_y < 1)
    {
        // Cannot work on less than a row
        return false;
    }
    auto max_x = grid.front().size();
    if (max_x < 1)
    {
        // Cannot work on less than a column
        return false;
    }

    auto total_cells = max_y * max_x;
    auto total_obstacles = static_cast<size_t>(static_cast<float>(total_cells) * (obstacle_fraction / 100));

    // For the amount of obstacles we need to create, generate random coordinates and insert an obstacle
    for (int i = 0; i < total_obstacles; ++i)
    {
        auto x = rand_r(&seed) % max_x;
        auto y = rand_r(&seed) % max_y;
        grid[y][x] = true;
    }
    return true;
}
