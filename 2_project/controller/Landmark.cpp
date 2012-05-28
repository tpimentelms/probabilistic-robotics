#include "Landmark.hpp"

Landmark::Landmark()
{
}

Landmark::~Landmark()
{
}

void Landmark::printParticlesPositions()
{
	this->particles.printParticlesPositions();
}

void Landmark::printPosition()
{
	this->particles.printParticlesPositionsMeans();
}

vector<particle> Landmark::getParticles()
{
	return this->particles.getParticles();
}

void Landmark::setParticles(vector<particle> particles)
{
	this->particles.setParticles(particles);
}
