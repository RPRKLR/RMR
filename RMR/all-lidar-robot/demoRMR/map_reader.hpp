#ifndef MAP_READER_HPP
#define MAP_READER_HPP
#include <vector>
#include <memory>

typedef struct
{
    int x;
    int y;
    int distance;
} Point2d;

typedef struct
{
    Point2d point;
    std::vector<double> coordiantes;
} MapPoint;

typedef struct
{
    int number_of_points;
    std::vector<MapPoint> points;
} MapObject;

typedef struct
{
    MapObject wall;
    int number_of_objects;
    std::vector<MapObject> obstacle;
} MapArea;

class MapReader
{
public:
    MapReader();
    void readMap(std::shared_ptr<char> filename, MapArea &maps);
};

#endif // MAP_READER_HPP
