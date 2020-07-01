#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <iostream>
#include <cstdlib>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, is_initialized_(false)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

    // pose_xyt_t prevPose;
    if (!is_initialized_) {
        prevPose_ = pose;
    }

    MovingLaserScan adjustedScan(scan, prevPose_, pose);

    for (auto ray : adjustedScan) {
        Point<int> begin, end;

        begin = map.convertPointToCell(ray.origin);
        end   = map.convertPointToCell(ray.end);
        rasterizeRay(begin, end, map);

    }

    prevPose_ = pose;
    is_initialized_ = true;
}

void Mapping::updateOdds(int x, int y, OccupancyGrid& map, CellOdds change)
{
    CellOdds lt = map.logOdds(x, y);
    if (lt >= 125) {
      lt = 127;
      return;
    }
    if (lt <= -127) {
      lt = -128;
      return;
    }
    lt += change;
    map.setLogOdds(x, y, lt);
}

void Mapping::rasterizeRay(Point<int> begin, Point<int> end, OccupancyGrid& map)
{
    int dx = std::abs(end.x - begin.x);
    int dy = std::abs(end.y - begin.y);
    int sx = begin.x < end.x ? 1 : -1;
    int sy = begin.y < end.y ? 1 : -1;
    int err = dx - dy;
    int x = begin.x;
    int y = begin.y;
    int e2;
    const CellOdds incr = 3;
    const CellOdds decr = -1;

    // updateOdds(end.x, end.y, map, incr);

    while(x != end.x || y != end.y) {
        updateOdds(x, y, map, decr);
        e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }

    updateOdds(end.x, end.y, map, incr);
}
