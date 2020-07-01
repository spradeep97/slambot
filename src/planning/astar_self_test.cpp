#include <planning/astar.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>
#include <iostream>

using namespace std;

int main(int argc, char* argv) {
  pose_xyt_t start{0}, goal{0};
  goal.x = -4.0;
  goal.y = 4.0;
  //OccupancyGrid map(15.0, 15.0, 0.05);
  OccupancyGrid map;
  map.loadFromFile("../data/astar/convex.map");
  ObstacleDistanceGrid distances;
  distances.setDistances(map);
  SearchParams params{0.01, 0.05, 1};
  AstarSearch astar(start, goal, distances, params);
  astar.print_node(astar.start_);
  astar.searchPath();
  cout << astar.goal_.pose_.x << " | " << astar.goal_.pose_.y << endl;
  return 0;
}
