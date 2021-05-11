#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <array>
#include <cmath>
#include <common/grid_utils.hpp>

#define MAX_VAL 10000
#define MAXIMUM_X 400
#define MAXIMUM_Y 400

using namespace std;



bool isValid(int x, int y, float dist, int X_MAX, int Y_MAX, double minDistanceToObstacle)
{
        if (dist < minDistanceToObstacle)
        {
            return false;
        }
        else
        {
            return (x < 0 || y < 0 || x >= X_MAX || y >= Y_MAX) ? false : true;
        }
}

bool isGoal(int x, int y, pose_xyt_t goal_cell)
{
    return (x == goal_cell.x && y == goal_cell.y) ? true : false;
}

double getGCost(int node_x, int node_y, int neighbor_x, int neighbor_y, double gCost)
{
    double dx = std::abs(neighbor_x - node_x);
    double dy = std::abs(neighbor_y - node_y);
    return (dx == 1 && dy == 1) ? gCost + 14 : gCost + 10;
}

double getHCost(int x, int y, pose_xyt_t goal_cell)
{
        double dx = std::abs(x - goal_cell.x);
        double dy = std::abs(y - goal_cell.y);
        return (dx >= dy) ? (14*dy + 10*(dx-dy)) : (14*dx + 10*(dy-dx));
}

double giveCostExp(float dist, double minDistanceToObstacle, double maxDistanceWithCost, double distanceCostExponent)
{
    return (dist > minDistanceToObstacle && dist < maxDistanceWithCost) ? pow(maxDistanceWithCost - dist, distanceCostExponent): 0.0;
}

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

    ///////////////////////////////////////// Init cells and more ////////////////////////
    robot_path_t path; //Contains all poses of path
    unsigned int X_MAX = distances.widthInCells();
    unsigned int Y_MAX = distances.heightInCells();

    double minObsDist = params.minDistanceToObstacle;
    double maxCostDist = params.maxDistanceWithCost;
    double distCostExp = params.distanceCostExponent;

    //Get cell coords from global coords
    pose_xyt_t start_cell, goal_cell;
    // setCellCoords(&start_cell2, start, goal, distances);
    // setCellCoords(&goal_cell2, start, goal, distances);
    Point<int> temp_xy;
    temp_xy = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    start_cell.x = temp_xy.x;
    start_cell.y = temp_xy.y;
    temp_xy = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);
    goal_cell.x = temp_xy.x;
    goal_cell.y = temp_xy.y;


    /////////////////////////////////////// Do Astar Search //////////////////////////////
    vector<Node> open;
    bool closed[X_MAX][Y_MAX];      //CL array for cells
    Node nodes[X_MAX][Y_MAX];   //nodes array  //CHECK THIS CONDITION AGAIN
    bool goalReached = false;

    //Initialize all node values to max
    for (int x = 0; x < X_MAX; x++) {
        for (int y = 0; y < Y_MAX; y++) {
            nodes[x][y].x = x;
            nodes[x][y].y = y;
            nodes[x][y].parentX = -1;
            nodes[x][y].parentY = -1;
            nodes[x][y].hCost = MAX_VAL;
            nodes[x][y].gCost = MAX_VAL;
            nodes[x][y].fCost = MAX_VAL;
            closed[x][y] = false;
        }
    }

    //Initialize our start node list
    int x = start_cell.x;
    int y = start_cell.y;
    nodes[x][y].hCost = 0.0;
    nodes[x][y].gCost = 0.0;
    nodes[x][y].fCost = 0.0;
    nodes[x][y].parentX = x;
    nodes[x][y].parentY = y;

    //Add start node to OL
    open.emplace_back(nodes[x][y]);

    while (!open.empty() && open.size() < X_MAX * Y_MAX)  //CHECK THIS CONDITION
    {
        if(goalReached) break;

        //Find node of lowest total cost (fCost) in OL and pop it
        float temp_cost = MAX_VAL + 1;
        int smallest = 0;
        for (int i = 0; i < open.size(); i++)
        {
            if (open[i].fCost < temp_cost)
            {
                temp_cost = open[i].fCost; //smallest cost-element value
                smallest = i; //smallest cost-element idx
            }
        }
        Node node = open[smallest];
        open.erase(open.begin() + smallest);

        //Add node to CL
        x = node.x;
        y = node.y;
        closed[x][y] = true;

        //For each neighbour [start at top left in cw direction]
        for (int offset_x = -1; offset_x <= 1; offset_x++)
        {
            if(goalReached) break;

            for (int offset_y = -1; offset_y <= 1; offset_y++)
            {
                if(goalReached) break;

                double gNew, hNew, fNew;
                int neighbor_x = x + offset_x;
                int neighbor_y = y + offset_y;

                if(neighbor_x >= 0 && neighbor_y >= 0 && isValid(neighbor_x, neighbor_y, distances(neighbor_x, neighbor_y), X_MAX, Y_MAX, minObsDist)) //CHECK THIS CONDITION
                {
                    if (isGoal(neighbor_x, neighbor_y, goal_cell)) //We found the goal pose!!!!
                    {
                        //Destination found (Found path)
                        nodes[neighbor_x][neighbor_y].parentX = x;
                        nodes[neighbor_x][neighbor_y].parentY = y;
                        goalReached = true;

                        //Make path
                        int goal_x = goal_cell.x;
                        int goal_y = goal_cell.y;

                        pose_xyt_t pose;
                        Node current;
                        Node previous;

                        pose.x = goal.x;
                        pose.y = goal.y;
                        pose.theta = goal.theta;
                        path.path.insert(path.path.begin(), pose); //Append goal to path

                        while ((nodes[goal_x][goal_y].parentX != nodes[goal_x][goal_y].x) || (nodes[goal_x][goal_y].parentY != nodes[goal_x][goal_y].y))
                        {
                            current = nodes[goal_x][goal_y];
                            goal_x = (int) nodes[goal_x][goal_y].parentX;
                            goal_y = (int) nodes[goal_x][goal_y].parentY;
                            previous = nodes[goal_x][goal_y];

                            if(current.x != previous.x && current.y != previous.y) // excluding start point
                            {
                                temp_xy = grid_position_to_global_position(Point<double>(previous.x,previous.y),distances);
                                pose.x = temp_xy.x;
                                pose.y = temp_xy.y;
                                pose.theta = atan2((current.y - previous.y), (current.x - previous.x));
                                path.path.insert(path.path.begin(), pose);
                            }
                        }
                    }
                    else if (!closed[neighbor_x][neighbor_y]) //We have unexplored nodes (cells) still
                    {
                        gNew = getGCost(x, y, neighbor_x, neighbor_y, node.gCost);
                        hNew = getHCost(neighbor_x, neighbor_y, goal_cell);
                        double expCost = giveCostExp(distances(neighbor_x, neighbor_y), minObsDist, maxCostDist, distCostExp);
                        fNew = gNew + hNew + expCost;

                        //If current total cost is uninitialized or..//..if its higher than the newly computed cost
                        if (nodes[neighbor_x][neighbor_y].fCost == MAX_VAL || nodes[neighbor_x][neighbor_y].fCost > fNew)
                        {   //Basically- is this path better than the existing path? If yes, do below
                            nodes[neighbor_x][neighbor_y].hCost = hNew;         //Replace previous high costs with new low costs
                            nodes[neighbor_x][neighbor_y].gCost = gNew;
                            nodes[neighbor_x][neighbor_y].fCost = fNew;
                            nodes[neighbor_x][neighbor_y].parentX = x;          //Set root node as the parent of this neighboring node
                            nodes[neighbor_x][neighbor_y].parentY = y;
                            open.emplace_back(nodes[neighbor_x][neighbor_y]);   //Add neighbors to OL
                        }
                    }
                }
            }
        }
    }

    ////////////////////////////////////// Finished Astar Search | Return path ///////////
    if (!goalReached) (cout << "No path found to goal... :((" << endl);
    path.utime = start.utime;
    path.path.insert(path.path.begin(),start);
    path.path_length = path.path.size();
    return path;
}

// void setCellCoords(pose_xyt_t *final_cell, pose_xyt_t start, pose_xyt_t goal,
                      // const ObstacleDistanceGrid& distances)
// {
//     Point<int> temp_xy;
//     temp_xy = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
//     final_cell->x = temp_xy.x;
//     final_cell->y = temp_xy.y;
//     // cout<< "temp: "<< temp_xy.x <<endl;
//     // cout<< "temp: "<< temp_xy.y <<endl;
// }
