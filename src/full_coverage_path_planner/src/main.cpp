//
// Created by ashwin on 05/11/21.
//

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "full_coverage_path_planner/boustrophedon_stc.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <list>
#include <rviz_visual_tools/rviz_visual_tools.h>

/**
 * Draw a nested vector of bools into an openCV image
 * @param grid
 * @return 2D 8-bit single-channel image
 */
cv::Mat drawMap(std::vector<std::vector<bool> > const& grid)
{
    int y_size = static_cast<int>(grid.size());
    int x_size = static_cast<int>(grid[0].size());

    cv::Mat mapImg = cv::Mat::zeros(y_size, x_size, CV_8U);  // CV_8U 8bit unsigned int 1 channel
    for (int k = 0; k < y_size; k++)
    {
        for (int l = 0; l < x_size; l++)
        {
            if (grid[k][l])
            {
                cv::rectangle(mapImg, {l, k}, {l, k}, 255);  // NOLINT
            }
        }
    }
    return mapImg;
}

/**
 * Draw path on a copy of the map
 * This is done twice: one to serve as input for calcDifference and another is returned for visualisation purposes
 * @param mapImg original map with just obstacles
 * @param pathImg Image that will feed into calcDifference
 * @param start Where does the path start?
 * @param path the actual path to be drawn
 * @return 2D RGB image for visualisation purposes
 */
cv::Mat drawPath(const cv::Mat &mapImg,
                 const cv::Mat &pathImg,
                 const Point_t &start,
                 std::list<Point_t> &path)
{
    cv::Mat pathViz = cv::Mat::zeros(mapImg.cols, mapImg.rows, CV_8UC3);
    std::vector<cv::Mat> channels;
    channels.push_back(mapImg.clone());
    channels.push_back(mapImg.clone());
    channels.push_back(mapImg.clone());
    cv::merge(channels, pathViz);

    int step = 0;
    for (auto it = path.begin(); it != path.end(); ++it)
    {
//      std::cout << "Path at (" << it->x << ", " << it->y << ")" << std::endl;
        cv::rectangle(pathImg, {it->x, it->y}, {it->x, it->y}, 255);  // NOLINT

        // Color the path in lighter and lighter color towards the end
        step++;
        int value = ((step * 200) / static_cast<int>(path.size())) + 50;
        cv::Scalar color(value, 128, 128);
        cv::rectangle(pathViz, {it->x, it->y}, {it->x, it->y}, color);  // NOLINT
    }

    // Draw the start and end in green and red, resp.
    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);
    cv::rectangle(pathViz,
                  {start.x, start.y},
                  {start.x, start.y},
                  green);
    cv::rectangle(pathViz,
                  {path.back().x, path.back().y},
                  {path.back().x, path.back().y},
                  red);
    return pathViz;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    auto occ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(30));

    std::vector<std::vector<bool>> grid;
    full_coverage_path_planner::BoustrophedonSTC planner;
    Point_t scaled;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 2;
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1;
    float r = 0.17;
    planner.parseGrid(*occ, grid, r, r, pose, scaled);

    int multiple_pass_counter, visited_counter;
    std::list<Point_t> path = full_coverage_path_planner::BoustrophedonSTC::boustrophedon_stc(grid,
                                                                                              scaled,
                                                                                              multiple_pass_counter,
                                                                                              visited_counter);
    std::vector<geometry_msgs::PoseStamped> plan;
    planner.parsePointlist2Plan(pose, path, plan);

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map","/rviz_visual_markers"));
    std::vector<geometry_msgs::Pose> planNew;
    planNew.reserve(plan.size());
    std::for_each(plan.begin(), plan.end(), [&planNew](geometry_msgs::PoseStamped &it) {planNew.push_back(it.pose);});
    visual_tools_->publishPath(planNew);
//    std::cout << plan.size() << std::endl;
//    std::cout << grid.size()*grid[0].size() << std::endl;
    while (1) {
        visual_tools_->trigger();
        ros::spinOnce();
    }

//    cv::Mat mapImg = drawMap(grid);
//    std::cout << grid.size() << std::endl;
//    cv::Mat pathImg = mapImg.clone();
//    cv::Mat pathViz = drawPath(mapImg, pathImg, scaled, path);
//    cv::imwrite("/tmp/testMap.png", pathViz);

    return EXIT_SUCCESS;
}