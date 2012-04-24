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
mat createAt();
mat createBt();
mat createUt();
mat createMu();

void kalmanFilter();

double randomGaussianNoise(double sigma, double mean);

#endif
