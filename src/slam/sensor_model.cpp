#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <math.h>

SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

Point<int> SensorModel::find_cell_along(const adjusted_ray_t& ray, const OccupancyGrid& map, double dist) {
    Point<double> neighbour;
    neighbour.x = ray.origin.x + (ray.range + dist) * cos(ray.theta);
    neighbour.y = ray.origin.y + (ray.range + dist) * sin(ray.theta);
    return global_position_to_grid_cell(neighbour, map);
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    double scanScore = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    for (const auto& ray : movingScan) {
        double rayScore = scoreRay(ray, map);
        scanScore += rayScore;
    }
    return scanScore;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    Point<int> end_cell = global_position_to_grid_cell(ray.end, map);

    double one_neigh_dist = map.metersPerCell() * std::sqrt(2.0);
    double two_neigh_dist = 2 * one_neigh_dist;

    Point<int> next_cell = find_cell_along(ray, map, one_neigh_dist);
    Point<int> prev_cell = find_cell_along(ray, map, -one_neigh_dist);
    Point<int> next2_cell = find_cell_along(ray, map, two_neigh_dist);
    Point<int> prev2_cell = find_cell_along(ray, map, -two_neigh_dist);
    Point<int> next3_cell = find_cell_along(ray, map, 3*one_neigh_dist);
    Point<int> prev3_cell = find_cell_along(ray, map, -3*one_neigh_dist);
    Point<int> next4_cell = find_cell_along(ray, map, 2*two_neigh_dist);
    Point<int> prev4_cell = find_cell_along(ray, map, -2*two_neigh_dist);

    CellOdds odds = map.logOdds(end_cell.x, end_cell.y);
    if (odds > 0) {
        return (double)odds;
    }
    else {
        //TODO: Add fractional logOdds.
        double frac1 = 1.0 / 2.0;
        double frac2 = 4.0 / 16.0;
        double oddsNext = (double)map.logOdds(next_cell.x, next_cell.y);
        double oddsPrev = (double)map.logOdds(prev_cell.x, prev_cell.y);
        double oddsNext2 = (double)map.logOdds(next2_cell.x, next2_cell.y);
        double oddsPrev2 = (double)map.logOdds(prev2_cell.x, prev2_cell.y);
        double tot_odds = 0.0;

        if (oddsNext > 0) tot_odds += frac1 * oddsNext;
        if (oddsPrev > 0) tot_odds += frac1 * oddsPrev;
        if (oddsNext2 > 0) tot_odds += frac2 * oddsNext2;
        if (oddsPrev2 > 0) tot_odds += frac2 * oddsPrev2;

        // if(oddsNext > 0 && oddsPrev > 0) std::cout << "Fractional log odds : " << oddsNext << " | " << oddsPrev << std::endl;

        return tot_odds+odds;
    }
}
