#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Robot.hpp>
#include <Map.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

#define RAIO 	0.75

using namespace std;
using namespace PlayerCc;
using namespace arma;

struct walls_found
{
	double distance;
	double angle;
};
typedef struct walls_found wallsFound; 

int main();

void move(double newV, double newW);

void sense();

mat predictMean();
mat predictionCov(mat A);
mat createAt();
mat createBt();
mat createUt();
mat createMu();
mat createRt();

void kalmanFilter();

bool interpretMeasurements();
vector<wallsFound> findLine();
pair<bool, point> findCorner(vector<wallsFound> lines);
pair<bool, point> findLandmark();

double randomGaussianNoise(double sigma, double mean);
double getMeanRoundWorld(vector<double> array, int worldSize);
double getMean(vector<double> array);
double getBetterAngle (unsigned int sensorUsed, double lineTheta);
vector<wallsFound> getLines(vector<double> cosMeans, vector<double> getPositions, vector<double> laserMeasurements, vector<double> distanceMeans);
pair<int, int> findLandmarkClusterOfMeasures(vector<int> validLaserMeasurements, vector<double> laserMeasurements);

void strategy();
void followWall(vector<wallsFound> lines);

#endif
