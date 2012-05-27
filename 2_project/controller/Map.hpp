#ifndef MAP_HPP
#define MAP_HPP

#include <Controller.hpp>
#include <Robot.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

using namespace std;
using namespace PlayerCc;
using namespace arma;

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
        
        void addWall(point begin, point end);
        void addCorner(point newCorner);
        void addLandmark(point newLandmark);
        
        void createKnownMap();
        
        vector<wall> getWalls();
        vector<point> getCorners();
        vector<point> getLandmarks();
        
    private:
        vector<wall> walls;
        vector<point> corners;
        vector<point> landmarks;
};

#endif
