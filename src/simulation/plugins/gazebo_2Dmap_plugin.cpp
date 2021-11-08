/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Adapted from: https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin
 * With very minor changes
 */

#include "simulation/gazebo_2Dmap_plugin.h"

#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Time.hh>
#include <string>
#include <vector>

#define PLUGIN_UNUSED(x) (void)x

namespace gazebo
{

OccupancyMapFromWorld::~OccupancyMapFromWorld() = default;

void OccupancyMapFromWorld::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    world_ = _parent;

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map2d", 1, true);
    map_service_ =
        nh_.advertiseService("gazebo_2Dmap_plugin/generate_map", &OccupancyMapFromWorld::ServiceCallback, this);
    PLUGIN_UNUSED(map_service_);
    map_resolution_ = 0.1;

    if (_sdf->HasElement("map_resolution"))
        map_resolution_ = _sdf->GetElement("map_resolution")->Get<double>();

    map_height_ = 0.3;

    if (_sdf->HasElement("map_height"))
        map_height_ = _sdf->GetElement("map_height")->Get<double>();

    init_robot_x_ = 0.0;

    if (_sdf->HasElement("init_robot_x"))
        init_robot_x_ = _sdf->GetElement("init_robot_x")->Get<double>();

    init_robot_y_ = 0.0;

    if (_sdf->HasElement("init_robot_y"))
        init_robot_y_ = _sdf->GetElement("init_robot_y")->Get<double>();

    map_size_x_ = 10.0;

    if (_sdf->HasElement("map_size_x"))
        map_size_x_ = _sdf->GetElement("map_size_x")->Get<double>();

    map_size_y_ = 10.0;

    if (_sdf->HasElement("map_size_y"))
        map_size_y_ = _sdf->GetElement("map_size_y")->Get<double>();
}

bool OccupancyMapFromWorld::ServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    CreateOccupancyMap();
    return true;
}

bool OccupancyMapFromWorld::worldCellIntersection(const vector3d& cell_center, const double cell_length,
                                                  const gazebo::physics::RayShapePtr& ray)

{
    // check for collisions with rays surrounding the cell
    //    ---
    //   | + |
    //    ---

    double dist;
    std::string entity_name;

    int cell_length_steps = 10;
    double side_length;

    // check for collisions with beams at increasing sizes to capture smaller
    // objects inside the cell
    for (int step = 1; step <= cell_length_steps; step++)
    {
        side_length = cell_length / cell_length_steps * step;

        for (int i = -1; i < 2; i += 2)
        {
#if GAZEBO_MAJOR_VERSION >= 9
            double start_x = cell_center.X() + i * side_length / 2;
            double start_y = cell_center.Y() - i * side_length / 2;

            for (int j = -1; j < 2; j += 2)
            {
                double end_x = cell_center.X() + j * side_length / 2;
                double end_y = cell_center.Y() + j * side_length / 2;

                ray->SetPoints(vector3d(start_x, start_y, cell_center.Z()), vector3d(end_x, end_y, cell_center.Z()));
#else
            double start_x = cell_center.x + i * side_length / 2;
            double start_y = cell_center.y - i * side_length / 2;

            for (int j = -1; j < 2; j += 2)
            {
                double end_x = cell_center.x + j * side_length / 2;
                double end_y = cell_center.y + j * side_length / 2;

                ray->SetPoints(vector3d(start_x, start_y, cell_center.z), vector3d(end_x, end_y, cell_center.z));
#endif
                ray->GetIntersection(dist, entity_name);

                if (!entity_name.empty())
                    return true;
            }
        }
    }

    return false;
}

void OccupancyMapFromWorld::cell2world(unsigned int cell_x, unsigned int cell_y, double map_size_x, double map_size_y,
                                       double map_resolution, double& world_x, double& world_y)
{
    world_x = cell_x * map_resolution - map_size_x / 2 + map_resolution / 2;
    world_y = cell_y * map_resolution - map_size_y / 2 + map_resolution / 2;
}

void OccupancyMapFromWorld::world2cell(double world_x, double world_y, double map_size_x, double map_size_y,
                                       double map_resolution, unsigned int& cell_x, unsigned int& cell_y)
{
    cell_x = static_cast<int>((world_x + map_size_x / 2) / map_resolution);
    cell_y = static_cast<int>((world_y + map_size_y / 2) / map_resolution);
}

bool OccupancyMapFromWorld::cell2index(int cell_x, int cell_y, unsigned int cell_size_x, unsigned int cell_size_y,
                                       unsigned int& map_index)
{
    if (cell_x >= 0 && cell_x < static_cast<int>(cell_size_x) && cell_y >= 0 && cell_y < static_cast<int>(cell_size_y))
    {
        map_index = cell_y * cell_size_y + cell_x;
        return true;
    }
    else
    {
        // return false when outside map bounds
        return false;
    }
}

bool OccupancyMapFromWorld::index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
    cell_y = index / cell_size_y;
    cell_x = index % cell_size_x;

    if (cell_x >= 0 && cell_x < cell_size_x && cell_y >= 0 && cell_y < cell_size_y)
        return true;
    else
    {
        // return false when outside map bounds
        return false;
    }
}

void OccupancyMapFromWorld::CreateOccupancyMap()
{
    // TODO(ashwin) map origin different from (0,0)
    vector3d map_origin(0, 0, map_height_);

    auto cells_size_x = static_cast<unsigned int>(map_size_x_ / map_resolution_);
    auto cells_size_y = static_cast<unsigned int>(map_size_y_ / map_resolution_);

    occupancy_map_ = new nav_msgs::OccupancyGrid();
    occupancy_map_->data.resize(cells_size_x * cells_size_y);
    // all cells are initially unknown
    std::fill(occupancy_map_->data.begin(), occupancy_map_->data.end(), -1);
    occupancy_map_->header.stamp = ros::Time::now();
    occupancy_map_->header.frame_id = "map";
    occupancy_map_->info.map_load_time = ros::Time(0);
    occupancy_map_->info.resolution = static_cast<float>(map_resolution_);
    occupancy_map_->info.width = cells_size_x;
    occupancy_map_->info.height = cells_size_y;
#if GAZEBO_MAJOR_VERSION >= 9
    occupancy_map_->info.origin.position.x = map_origin.X() - map_size_x_ / 2;
    occupancy_map_->info.origin.position.y = map_origin.Y() - map_size_y_ / 2;
    occupancy_map_->info.origin.position.z = map_origin.Z();
#else
    occupancy_map_->info.origin.position.x = map_origin.x - map_size_x_ / 2;
    occupancy_map_->info.origin.position.y = map_origin.y - map_size_y_ / 2;
    occupancy_map_->info.origin.position.z = map_origin.z;
#endif
    occupancy_map_->info.origin.orientation.w = 1;

    gazebo::physics::PhysicsEnginePtr engine = GetPhysicsPtr(world_);
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    // identify free space by spreading out from initial robot cell
    double robot_x = init_robot_x_;
    double robot_y = init_robot_y_;

    ROS_INFO_NAMED(name_, "Starting wavefront expansion from (%lf, %lf)", robot_x, robot_y);

    // find initial robot cell
    unsigned int cell_x, cell_y, map_index;
    world2cell(robot_x, robot_y, map_size_x_, map_size_y_, map_resolution_, cell_x, cell_y);

    if (!cell2index(cell_x, cell_y, cells_size_x, cells_size_y, map_index))
    {
        ROS_ERROR_NAMED(name_,
                        "initial robot pos is outside map, could not create "
                        "map");
        return;
    }

    std::vector<unsigned int> wavefront;
    wavefront.push_back(map_index);

    // wavefront expansion for identifying free, unknown and occupied cells
    while (!wavefront.empty())
    {
        map_index = wavefront.at(0);
        wavefront.erase(wavefront.begin());

        index2cell(map_index, cells_size_x, cells_size_y, cell_x, cell_y);

        // mark cell as free
        occupancy_map_->data.at(map_index) = 0;

        // explore cells neighbors in an 8-connected grid
        unsigned int child_index;
        double world_x, world_y;
        uint8_t child_val;

        // 8-connected grid
        for (int i = -1; i < 2; i++)
        {
            for (int j = -1; j < 2; j++)
            {
                // makes sure index is inside map bounds
                if (cell2index(cell_x + i, cell_y + j, cells_size_x, cells_size_y, child_index))
                {
                    child_val = occupancy_map_->data.at(child_index);

                    // only update value if cell is unknown
                    if (child_val != 100 && child_val != 0 && child_val != 50)
                    {
                        cell2world(cell_x + i, cell_y + j, map_size_x_, map_size_y_, map_resolution_, world_x, world_y);

                        bool cell_occupied =
                            worldCellIntersection(vector3d(world_x, world_y, map_height_), map_resolution_, ray);

                        if (cell_occupied)
                            // mark cell as occupied
                            occupancy_map_->data.at(child_index) = 100;

                        else
                        {
                            // add cell to wavefront
                            wavefront.push_back(child_index);
                            // mark wavefront in map so we don't add children to wavefront multiple
                            // times
                            occupancy_map_->data.at(child_index) = 50;
                        }
                    }
                }
            }
        }
    }

    map_pub_.publish(*occupancy_map_);
    ROS_INFO_NAMED(name_, "Occupancy map generation complete");
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo
