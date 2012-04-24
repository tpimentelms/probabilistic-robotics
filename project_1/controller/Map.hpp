#ifndef MAP_HPP
#define MAP_HPP

#include <ProcessLogger.h>
#include <vector>

struct point_t
{
    double x, y;
};
typedef struct point_t point;

struct wall_t
{
    point begin, end;
};
typedef struct wall_t wall;

class Map
{
    public:
        Map();
        ~Map();
        
        addWall(wall newWall);
        addCorner(point newCorner);
        addLandmark(point newLandmark);
        
        getWalls();
        getCorners();
        getLandmarks();
        
    private:
        vector<wall> walls;
        vector<point> corners;
        vector<point> landmarks;
};

#endif
