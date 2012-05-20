#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Robot.hpp>
#include <Landmark.hpp>
#include <Particles.hpp>
#include <Map.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

#define RAIO 	0.75

using namespace std;
using namespace PlayerCc;
using namespace arma;

typedef struct point_t point;

struct walls_found
{
	double distance;
	double angle;
};
typedef struct walls_found wallsFound; 

int main();

void move(double newV, double newW);

vector<wallsFound> sense();

void updateLandmarkState(point landmark);

void particleFilter(vector<wallsFound> lines);
vector<particle> predictParticles(vector<particle> robotParticles);
void updateParticles(vector<wallsFound> lines, vector<particle> robotParticles);
particle movementPrediction(particle particlePosition, double deltaT);

vector<wallsFound> interpretMeasurements();
vector<wallsFound> findLine();
pair<bool, point> findCorner(vector<wallsFound> lines);
pair<bool, point> findLandmark();

double randomGaussianNoise(double sigma, double mean);
double getMeanRoundWorld(vector<double> array, int worldSize);
double getMean(vector<double> array);
double getBetterAngle (unsigned int sensorUsed, double lineTheta);
vector<wallsFound> getLines(vector<double> cosMeans, vector<double> getPositions, vector<double> laserMeasurements, vector<double> distanceMeans);
pair<int, int> findLandmarkClusterOfMeasures(vector<int> validLaserMeasurements, vector<double> laserMeasurements);

void strategy(vector<wallsFound> lines);
void followWall(vector<wallsFound> lines, int wallFound);


long long int timeval_diff(struct timeval *difference, struct timeval *end_time, struct timeval *start_time);

#endif
