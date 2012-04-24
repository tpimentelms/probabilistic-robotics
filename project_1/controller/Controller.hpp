#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Robot.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>

using namespace std;
using namespace PlayerCc;

int main();

void move(double newV, double newW);

void sense();

void positionPredict();
void kalmanFilter();

double randomGaussianNoise(double sigma, double mean);

#endif
