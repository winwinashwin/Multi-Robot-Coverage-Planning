// Copyright [2021] Ashwin A Nayar

#ifndef FRONTIER_EXPLORATION_COSTMAP_TOOLS_H
#define FRONTIER_EXPLORATION_COSTMAP_TOOLS_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <queue>
#include <ros/ros.h>
#include <vector>

namespace frontier_exploration
{

std::vector<size_t> nhood4(size_t idx, const costmap_2d::Costmap2D& costmap)
{
    // get 4-connected neighbourhood indexes, check for edge of map
    std::vector<size_t> out;

    size_t sizeX = costmap.getSizeInCellsX(), sizeY = costmap.getSizeInCellsY();

    if (idx > sizeX * sizeY - 1)
    {
        ROS_WARN("Searching neighbourhood for off-map point");
        return out;
    }

    if (idx % sizeX > 0)
    {
        out.push_back(idx - 1);
    }
    if (idx % sizeX < sizeX - 1)
    {
        out.push_back(idx + 1);
    }
    if (idx >= sizeX)
    {
        out.push_back(idx - sizeX);
    }
    if (idx < sizeX * (sizeY - 1))
    {
        out.push_back(idx + sizeX);
    }
    return out;
}

std::vector<size_t> nhood8(size_t idx, const costmap_2d::Costmap2D& costmap)
{
    // get 8-connected neighbourhood indexes, check for edge of map
    std::vector<size_t> out = nhood4(idx, costmap);

    size_t sizeX = costmap.getSizeInCellsX(), sizeY = costmap.getSizeInCellsY();

    if (idx > sizeX * sizeY - 1)
    {
        return out;
    }

    if (idx % sizeX > 0 && idx >= sizeX)
    {
        out.push_back(idx - 1 - sizeX);
    }
    if (idx % sizeX > 0 && idx < sizeX * (sizeY - 1))
    {
        out.push_back(idx - 1 + sizeX);
    }
    if (idx % sizeX < sizeX - 1 && idx >= sizeX)
    {
        out.push_back(idx + 1 - sizeX);
    }
    if (idx % sizeX < sizeX - 1 && idx < sizeX * (sizeY - 1))
    {
        out.push_back(idx + 1 + sizeX);
    }

    return out;
}

bool nearestCell(size_t& outResult, size_t start, uint8_t val, const costmap_2d::Costmap2D& costmap)
{
    const uint8_t* map = costmap.getCharMap();
    const size_t sizeX = costmap.getSizeInCellsX(), sizeY = costmap.getSizeInCellsY();

    if (start >= sizeX * sizeY)
    {
        return false;
    }

    // initialize breadth first search
    std::queue<size_t> bfs;
    std::vector<bool> visited(sizeX * sizeY, false);

    // push initial cell
    bfs.push(start);
    visited[start] = true;

    // search for neighbouring cell matching value
    while (!bfs.empty())
    {
        size_t idx = bfs.front();
        bfs.pop();

        // return if cell of correct value is found
        if (map[idx] == val)
        {
            outResult = idx;
            return true;
        }

        // iterate over all adjacent unvisited cells
        for (auto nbr : nhood8(idx, costmap))
        {
            if (!visited[nbr])
            {
                bfs.push(nbr);
                visited[nbr] = true;
            }
        }
    }
    return false;
}
}  // namespace frontier_exploration
#endif  // FRONTIER_EXPLORATION_COSTMAP_TOOLS_H
