#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 

    double scan_score = 0.0;
    MovingLaserScan movingScan(scan, sample.pose, sample.pose);

    for (adjusted_ray_t ray : movingScan)
    {
        scan_score += scoreRay(ray, map);
    }

    return scan_score; // Placeholder
    
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell. 

    
    end_point = getRayEndPointOnMap(ray, map);

    Point<int> cell_pos;
    cell_pos = global_position_to_grid_cell(end_point, map);

    if (map.isCellInGrid(cell_pos.x, cell_pos.y))
    {
        CellOdds cell;
        cell = map.logOdds(cell_pos.x, cell_pos.y);
        return (cell>occupancy_threshold_) ? 1.0:0.0;
    }

    return 0.0

}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 

    Point<float> end_point = getRayEndPointOnMap(ray, map);
    vector<Point<int>> bfs_offsets_;
    bfs_offsets_.push_back(Point<int>(-1, -1));
    bfs_offsets_.push_back(Point<int>(-1, 0));
    bfs_offsets_.push_back(Point<int>(-1, 1));
    bfs_offsets_.push_back(Point<int>(0, -1));
    bfs_offsets_.push_back(Point<int>(0, 1));
    bfs_offsets_.push_back(Point<int>(1, -1));
    bfs_offsets_.push_back(Point<int>(1, 0));
    bfs_offsets_.push_back(Point<int>(1, 1));
    
    return Point<int>(0,0); // Placeholder
    
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    float x = ray.origin.x;
    float y = ray.origin.y;
    float range = ray.range;
    float theta = ray.theta;

    float x_end = x + range*cos(theta);
    float y_end = y + range*sin(theta);
    Point<float> end_point(x_end, y_end);

    return end_point; // Placeholder
    
}
