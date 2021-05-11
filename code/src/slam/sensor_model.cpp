#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel()
: metersPerCell_(0.05f)
// , kMaxLaserDistance_(maxLaserDistance)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    double scanScore = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double rayScore;

    for(const auto& ray : movingScan)
    {
        rayScore = scoreRay(ray, map);
        scanScore += rayScore;
    }

    return scanScore;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map){
    double rayScore = 0.0;
    // if(ray.range <= kMaxLaserDistance_)
    // {
    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
    Point<int> rayCell;
    double hitFraction = 0.5;
    bool is_hit = false;
    bool is_partial_hit = false;
    int partial_log_odds = 0;
    int log_odds = 0;

    rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    if(map.isCellInGrid(rayCell.x, rayCell.y))
    {
        log_odds = map(rayCell.x,rayCell.y);
        is_hit = isCellHit(log_odds);
        if (!is_hit)
        {   Point<int> after;
            Point<int> before;

            // generate after xy
            after.x = static_cast<int>(((ray.range + metersPerCell_) * std::cos(ray.theta) * metersPerCell_) + rayStart.x);
            after.y = static_cast<int>(((ray.range + metersPerCell_) * std::sin(ray.theta) * metersPerCell_) + rayStart.y);
            // generate before xy
            before.x = static_cast<int>(((ray.range - metersPerCell_) * std::cos(ray.theta) * metersPerCell_) + rayStart.x);
            before.y = static_cast<int>(((ray.range - metersPerCell_) * std::sin(ray.theta) * metersPerCell_) + rayStart.y);

            if (map.isCellInGrid(after.x, after.y))
            {
                partial_log_odds = map(after.x,after.y);
                is_partial_hit = isCellHit(partial_log_odds);
            }
            if (!is_partial_hit && map.isCellInGrid(before.x, before.y)) //else if?
            {
                partial_log_odds = map(before.x,before.y);
                is_partial_hit = isCellHit(partial_log_odds);
            }
        }
        // }
        if (is_hit) {
            rayScore = log_odds;
        } else if (is_partial_hit) {
            rayScore = hitFraction * log_odds;
        }
    }
    return rayScore;
}

double SensorModel::isCellHit(double log_odds) {
    return (log_odds > 0) ? true : false;
}

// float SensorModel::xCoord(float ray_range, float delta_range, float ray_theta, float rayStart_x) {
//     float x = static_cast<int>(((ray_range + delta_range)* std::cos(ray_theta) * map.cellsPerMeter()) + rayStart_x);
//     return x;
// }
//
// float SensorModel::yCoord(float ray_range, float delta_range, float ray_theta, float rayStart_y) {
//     float y = static_cast<int>(((ray_range + delta_range)* std::cos(ray_theta) * map.cellsPerMeter()) + rayStart_y);
//     return y;
// }

// rayCell.x = xCoord(ray.range, 0.0, ray.theta, rayStart.x);
// rayCell.y = yCoord(ray.range, 0.0, ray.theta, rayStart.y);
// after.x = xCoord(ray.range, metersPerCell(), ray.theta, rayStart.x);
// after.y = yCoord(ray.range, metersPerCell(), ray.theta, rayStart.y);
// before.x = xCoord(ray.range, -metersPerCell(), ray.theta, rayStart.x);
// before.y = yCoord(ray.range, -metersPerCell(), ray.theta, rayStart.y);
