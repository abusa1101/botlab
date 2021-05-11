#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs

    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path

    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};



struct Node
{
    double x;
    double y;
    double hCost;
    double gCost;
    double fCost;
    double parentX;
    double parentY;
};

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
*
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

bool isValid(int x, int y, float distances_x_y, int x_max, int y_max, double minDistanceToObstacle);

bool isGoal(int x, int y, pose_xyt_t goal_cell);

double getGCost(int node_x, int node_y, int neighbor_x, int neighbor_y, double gCost);

double getHCost(int x, int y, pose_xyt_t goal_cell);

double giveCostExp(float distances_x_y, double minDistanceToObstacle, double maxDistanceWithCost, double distanceCostExponent);

#endif // PLANNING_ASTAR_HPP
