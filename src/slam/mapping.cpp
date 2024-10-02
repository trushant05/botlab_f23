#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.

    for (adjusted_ray_t ray : movingScan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  
    float x = ray.origin.x;
    float y = ray.origin.y;
    float range = ray.range;
    float theta = ray.theta;

    float x_end = x + range*cos(theta);
    float y_end = y + range*sin(theta);
    Point<float> end_point(x_end, y_end);

    Point<int> cell_pos;
    cell_pos = global_position_to_grid_cell(end_point, map);
    
    if (map.isCellInGrid(cell_pos.x, cell_pos.y)){
        CellOdds cell;
        cell = map.logOdds(cell_pos.x, cell_pos.y);
        cell += kHitOdds_;
        map.setLogOdds(cell_pos.x, cell_pos.y, cell);
    }
    
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    std::vector<Point<int>> cells;
    cells = bresenham(ray, map);
    for (Point<int> cell : cells){
        if (map.isCellInGrid(cell.x, cell.y)){
            CellOdds cell_odds;
            cell_odds = map.logOdds(cell.x, cell.y);
            cell_odds -= kMissOdds_;
            map.setLogOdds(cell.x, cell.y, cell_odds);
        }
    }
    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    float x_float = ray.origin.x;
    float y_float = ray.origin.y;
    float range = ray.range;
    float theta = ray.theta;
    
    Point<int> cell_start;
    cell_start = global_position_to_grid_cell(ray.origin, map);

    float x_end = x_float + range*cos(theta);
    float y_end = y_float + range*sin(theta);
    Point<float> end_point(x_end, y_end);
    Point<int> cell_end;
    cell_end = global_position_to_grid_cell(end_point, map);

    int x0 = cell_start.x; int y0 = cell_start.y;
    int x1 = cell_end.x; int y1 = cell_end.y;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0; int y = y0;
    std::vector<Point<int>> cells;
    // cells.push_back(Point<int>(x, y));
    
    while (x!=x1 || y!=y1){
        int e2 = 2*err;
        if (e2 > -dy){
            err -= dy;
            x += sx;
        }
        if (e2 < dx){
            err += dx;
            y += sy;
        }
        cells.push_back(Point<int>(x, y));
    }
    
    return cells;
    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}
