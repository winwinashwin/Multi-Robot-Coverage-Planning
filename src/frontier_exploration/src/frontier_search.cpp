// Copyright [2021] Ashwin A Nayar

#include "frontier_exploration/frontier_search.h"

#include "frontier_exploration/costmap_tools.h"

#include <algorithm>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <limits>
#include <mutex>
#include <queue>
#include <vector>

namespace frontier_exploration
{
using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap, double potentialScale, double gainScale,
                               double minFrontierSize)
    : _costmap(costmap)
    , _map(nullptr)
    , _sizeX{}
    , _sizeY{}
    , _potentialScale(potentialScale)
    , _gainScale(gainScale)
    , _minFrontierSize(minFrontierSize)
{
}

std::vector<Frontier> FrontierSearch::searchFrom(const geometry_msgs::Point& position)
{
    uint mx, my;
    if (!_costmap->worldToMap(position.x, position.y, mx, my))
    {
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return {};
    }

    std::vector<Frontier> frontierList;

    // make sure map is consistent and locked for duration of search
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

    _map = _costmap->getCharMap();
    _sizeX = _costmap->getSizeInCellsX();
    _sizeY = _costmap->getSizeInCellsY();

    // mark all cells as unchecked
    std::vector<CellState> cellStates(_sizeX * _sizeY, UNCHECKED);

    // initialize breadth first search (outer)
    std::queue<size_t> bfs;

    // find the closest clear cell to start search
    size_t clear, pos = _costmap->getIndex(mx, my);
    if (nearestCell(clear, pos, FREE_SPACE, *_costmap))
    {
        bfs.push(clear);
    }
    else
    {
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }

    cellStates[bfs.front()] = MAP_OPEN;

    while (!bfs.empty())
    {
        size_t p = bfs.front();
        bfs.pop();

        if (cellStates[p] == MAP_CLOSED)
            continue;

        if (isFrontierCell(p))
        {
            // If we encountered a frontier cell, we run another bfs (inner BFS)
            // starting from this frontier cell to extract the complete frontier it is
            // a part of. By keeping track of the cell state we make sure no frontier
            // cell is assigned to multiple frontiers
            Frontier newFrontier = buildFrontier(p, pos, cellStates);
            // Threshold based on frontier size
            if (newFrontier.size * _costmap->getResolution() >= _minFrontierSize)
            {
                frontierList.push_back(newFrontier);
            }
        }
        // Iterate over neighbours of current point
        for (auto nbr : nhood8(p, *_costmap))
        {
            // Check neighbourhood of neighbour
            auto nHood = nhood8(nbr, *_costmap);
            bool hasFreeNeighbour = std::any_of(nHood.begin(), nHood.end(),
                                                [&map = this->_map](auto idx) { return map[idx] == FREE_SPACE; });
            // Include neighbour in search iff it has a free neighbour
            if (cellStates[nbr] != MAP_OPEN && cellStates[nbr] != MAP_CLOSED && hasFreeNeighbour)
            {
                bfs.push(nbr);
                cellStates[nbr] = MAP_OPEN;
            }
        }
        cellStates[p] = MAP_CLOSED;
    }

    // set costs of frontiers
    for (auto& frontier : frontierList)
    {
        frontier.cost = frontierCost(frontier);
    }
    // sort frontiers based on cost
    std::sort(frontierList.begin(), frontierList.end(),
              [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

    return frontierList;
}

Frontier FrontierSearch::buildFrontier(size_t initialCell, size_t reference, std::vector<CellState>& cellStates)
{
    Frontier output;
    std::vector<size_t> frontierIndices;

    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.minDistance = std::numeric_limits<double>::infinity();

    uint ix, iy;
    _costmap->indexToCells(initialCell, ix, iy);
    _costmap->mapToWorld(ix, iy, output.initial.x, output.initial.y);

    std::queue<size_t> bfs;
    bfs.push(initialCell);
    cellStates[initialCell] = FRONTIER_OPEN;

    uint rx, ry;
    double referenceX, referenceY;
    _costmap->indexToCells(reference, rx, ry);
    _costmap->mapToWorld(rx, ry, referenceX, referenceY);

    while (!bfs.empty())
    {
        auto p = bfs.front();
        bfs.pop();

        if (cellStates[p] == FRONTIER_CLOSED || cellStates[p] == MAP_CLOSED)
            continue;

        if (isFrontierCell(p))
        {
            uint mx, my;
            double wx, wy;
            _costmap->indexToCells(p, mx, my);
            _costmap->mapToWorld(mx, my, wx, wy);

            geometry_msgs::Point point;
            point.x = wx;
            point.y = wy;
            output.points.push_back(point);

            // update frontier size
            output.size++;

            // update centroid of frontier
            output.centroid.x += wx;
            output.centroid.y += wy;

            // determine frontier's distance from robot, going by closest grid cell
            // to robot
            double distance = sqrt(pow((static_cast<double>(referenceX) - static_cast<double>(wx)), 2.0) +
                                   pow((static_cast<double>(referenceY) - static_cast<double>(wy)), 2.0));
            if (distance < output.minDistance)
            {
                output.minDistance = distance;
                output.middle.x = wx;
                output.middle.y = wy;
            }

            // add to queue for breadth first search
            bfs.push(p);
            frontierIndices.push_back(p);
            for (auto nbr : nhood8(p, *_costmap))
            {
                if (cellStates[nbr] == MAP_OPEN || cellStates[nbr] == UNCHECKED)
                {
                    bfs.push(nbr);
                    cellStates[nbr] = FRONTIER_OPEN;
                }
            }
        }
        cellStates[p] = FRONTIER_CLOSED;
    }
    // mark all frontier points as processed
    std::for_each(frontierIndices.begin(), frontierIndices.end(),
                  [&cellStates](auto idx) { cellStates[idx] = MAP_CLOSED; });

    output.centroid.x /= output.size;
    output.centroid.y /= output.size;

    return output;
}

bool FrontierSearch::isFrontierCell(size_t idx)
{
    if (_map[idx] != NO_INFORMATION)
        return false;
    auto nHood = nhood4(idx, *_costmap);

    return std::any_of(nHood.begin(), nHood.end(), [map = this->_map](auto idx) { return map[idx] == FREE_SPACE; });
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
    return (_potentialScale * frontier.minDistance * _costmap->getResolution()) -
           (_gainScale * frontier.size * _costmap->getResolution());
}
}  // namespace frontier_exploration
