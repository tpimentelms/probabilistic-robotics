#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Robot.hpp>
#include <ProcessLogger.h>
#include <libplayerc++/playerc++.h>

using namespace std;
using namespace PlayerCc;

int main();

void move(double newV, double newW);
void sense();

double randomGaussianNoise(double sigma, double mean);

#endif
