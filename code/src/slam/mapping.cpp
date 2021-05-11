#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_){
        previousPose_ = pose;
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);

    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
    }

    for(auto& ray : movingScan){
        scoreRay(ray, map);
    }

    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range <= kMaxLaserDistance_)
    {
        // Point<int> rayStart = global_position_to_grid_cell(ray.origin, map)
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);
        // trying to drop the floating numbers- can also do std::float instead of static cast

        if(map.isCellInGrid(rayCell.x, rayCell.y)) {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range <= kMaxLaserDistance_)
    {
        // Point<int> rayStart = global_position_to_grid_cell(ray.origin, map)
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        // can also do std::float instead of static cast I think
        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)) {
            //Implement Bresenham's algorithm here I think
            int x0 = (int) rayStart.x;
            int y0 = (int) rayStart.y;
            int x1 = rayCell.x;
            int y1 = rayCell.y;
            // printf("x0 y0, x1 y1: %d %d %d %d\n", x0, y0, x1, y1);

            int dx = abs(x1 - x0);
            int dy = abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;
            int x = x0;
            int y = y0;

            while(x != x1 || y != y1){
                // printf("x y: %d %d\n", x, y);
                if(map.isCellInGrid(x, y)) {
                    decreaseCellOdds(x, y, map);
                    int e2 = 2*err;
                    if (e2 >= -dy){
                        err -= dy;
                        x += sx;
                    }
                    if (e2 <= dx){
                        err += dx;
                        y += sy;
                    }
                }

            }
        }
    }
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map) {
    if(!initialized_) {
        //do nothing
    }
    //current cell not at max
    // if(map(x,y) > 255 - kHitOdds_) OR
    else if(127 - map(x,y) > kHitOdds_) {
        map(x,y) += kHitOdds_;
    }
    //current cell saturated already
    else {
        map(x,y) = 127;
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map) {
    if(!initialized_) {
        //do nothing
    }
    //current cell not at min
    else if(map(x,y) - (-127) > kMissOdds_) {
        map(x,y) -= kMissOdds_;
    }
    //current cell saturated already
    else {
        map(x,y) = (-127);
    }
}
