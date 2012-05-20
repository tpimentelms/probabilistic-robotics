#include "Landmark.hpp"

Landmark::Landmark()
{
}

Landmark::~Landmark()
{
}

void Landmark::printLandmarkParticlesPositions()
{
	this->landmarkParticles.printParticlesPositions();
}

void Landmark::printLandmarkPosition()
{
	this->landmarkParticles.printParticlesPositionsMeans();
}
