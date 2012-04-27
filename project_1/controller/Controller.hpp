#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Robot.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

using namespace std;
using namespace PlayerCc;
using namespace arma;

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
int findLine();
bool findCorner();
bool findLandmark();

double randomGaussianNoise(double sigma, double mean);
double getMeanRoundWorld(vector<double> array, int worldSize);
double getMedian(vector<double> array);

void strategy();

#endif
