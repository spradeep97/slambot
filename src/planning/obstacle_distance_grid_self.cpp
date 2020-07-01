#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

float ObstacleDistanceGrid::getDistance(int x, int y)
{
    if(isCellInGrid(x, y)) {
        return operator()(x, y);
    }
    return 0;
}

void ObstacleDistanceGrid::setDistance(int x, int y, float value)
{
    if (isCellInGrid(x, y)) {
        operator()(x, y) = value;
    }
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    resetGrid(map);

    //Thresholding the map
    for (int i = 0; i < map.widthInCells(); ++i) {
        for (int j = 0; j < map.heightInCells(); ++j) {
            if (map.logOdds(i,j) > 0) {
                setDistance(i, j, 0.0);
                Point<int> p(i,j);
                queue_.push(p);
            } else if (map.logOdds(i,j) < 0) {
                setDistance(i, j, -1.0);
            }
        }
    }

    //Visit neighbours
    while (!queue_.empty()) {
        visit_neighbours(queue_.front());
        queue_.pop();
    }
}

void ObstacleDistanceGrid::visit_neighbours(Point<int>& p)
{
    int r = p.x;
    int c = p.y;
    float w = getDistance(r, c);
    Point<int> p1(-1,0), p2(0,-1), p3(0,1), p4(1,0);
    Point<int> neighbours[4] = {p1, p2, p3, p4};
    for (auto& p : neighbours) {
        int row = r + p.x;
        int col = c + p.y;
        if (isCellInGrid(row, col)) {
            if (getDistance(row, col) < 0) {
                setDistance(row, col, w + 0.1);
                Point<int> p(row, col);
                queue_.push(p);
            }
        }
    }
}

// void ObstacleDistanceGrid::print_distances()
// {
//     for (int i = 0; i < map.heightInCells(); ++i) {
//         for (int j = 0; j < map.widthInCells(); ++j) {
//             std::cout << getDistance(j,i) << "|";
//         }
//         std::cout << "\n";
//     }
// }

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();

    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }

    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();

    cells_.resize(width_ * height_);
}
