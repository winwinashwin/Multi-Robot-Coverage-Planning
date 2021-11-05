//
// Created by ashwin on 04/11/21.
//

#ifndef CATKIN_WS_BOUSTROPHEDON_STC_H
#define CATKIN_WS_BOUSTROPHEDON_STC_H

#include <list>
#include "coverage_planning/types.h"
#include <vector>

namespace coverage_planning {
    class BoustrophedonSTC
    {
    public:
        /**
         * Find a path that does the boustrophedon pattern starting from init until a dead end is reached in the grid
         * @param grid 2D grid of bools. true == occupied/blocked/obstacle
         * @param init start position
         * @param visited all the nodes visited by the boustrophedon pattern
         * @return list of nodes that form the boustrophedon pattern
         */
        static std::list<gridNode_t> boustrophedon(std::vector<std::vector<bool> > const &grid, std::list<gridNode_t> &init,
                                                   std::vector<std::vector<bool> > &visited);

// ????????? Why is init a list?
        /**
         * Perform Boustrophedon-STC (Spanning Tree Coverage) coverage path planning.
         * In essence, the robot moves forward until an obstacle or visited node is met, then turns right or left (making a boustrophedon pattern)
         * When stuck in the middle of the boustrophedon, use A* to get out again and start a new boustrophedon, until a* can't find a path to uncovered cells
         * @param grid
         * @param init
         * @return
         */
        static std::list<Point_t> boustrophedon_stc(std::vector<std::vector<bool> > const &grid,
                                                    Point_t &init,
                                                    int &multiple_pass_counter,
                                                    int &visited_counter);

    private:
        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        // ?????????? How does this work again?
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief  Initialization function for the FullCoveragePathPlanner object
         * @param  name The name of this planner
         * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    };
}
#endif //CATKIN_WS_BOUSTROPHEDON_STC_H
