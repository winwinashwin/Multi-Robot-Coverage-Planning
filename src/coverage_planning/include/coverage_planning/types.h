//
// Created by ashwin on 04/11/21.
//

#ifndef CATKIN_WS_TYPES_H
#define CATKIN_WS_TYPES_H

#include <geometry_msgs/Point.h>

namespace coverage_planning {
    using Point_t = geometry_msgs::Point;

    typedef struct gridNode_t {
        Point_t pos;

        /** Path cost
         * cost of the path from the start node to gridNode_t
         */
        int cost{};

        /** Heuristic cost
         * cost of the cheapest path from this gridNode_t to the goal
         */
        int he{};
    } gridNode_t;

    enum {
        point = 0,
        east = 1,
        west = 2,
        north = 3,
        south = 4
    };
}
#endif //CATKIN_WS_TYPES_H
