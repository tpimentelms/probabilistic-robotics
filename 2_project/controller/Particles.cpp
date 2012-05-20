#include "Particles.hpp"

Particles::Particles()
{
	static bool first = true;
	if (first == true)
	{
		srand(time(0));
		first = false;
	}
	
	unsigned int counter;
	this->particleArray.resize(500);
	particle startingParticle;
	
	for (counter = 0; counter < this->particleArray.size(); counter++)
	{
		startingParticle.x = double(rand() % 2700 - 1300)/100;
		startingParticle.y = double(rand() % 2500 - 1200)/ 100;
		startingParticle.th = double(rand() % 1800)/50;
		this->particleArray[counter] = startingParticle;
	}
}

Particles::~Particles()
{
}

void Particles::printParticlesPositions()
{
	if (this->particleArray.size() == 0)
	{
		LOG(LEVEL_FATAL) << "Particle info";
		LOG(LEVEL_FATAL) << "Trying to access particles data while "
                         << "array size is zero.";
		
		return;
	}
	
	LOG(LEVEL_WARN) << "Particles info";
	LOG(LEVEL_INFO) << "Particles array size = " << this->particleArray.size();
	
	unsigned int counter;
	for (counter = 0; counter < this->particleArray.size(); counter++)
	{	
		LOG(LEVEL_INFO) << "Particle[" << counter << "].x = " << this->particleArray.at(counter).x;
		LOG(LEVEL_INFO) << "Particle[" << counter << "].y = " << this->particleArray.at(counter).y;
		LOG(LEVEL_INFO) << "Particle[" << counter << "].th = " << this->particleArray.at(counter).th;
	}
}

void Particles::printParticlesPositionsMeans()
{
	double meanX = 0;
	double meanY = 0;
	double meanTh = 0;
	
	if (this->particleArray.size() == 0)
	{
		LOG(LEVEL_FATAL) << "Particle info";
		LOG(LEVEL_FATAL) << "Trying to access particles data while "
                         << "array size is zero.";
		
		return;
	}
	
	LOG(LEVEL_WARN) << "Particles info";
	LOG(LEVEL_INFO) << "Particles array size = " << this->particleArray.size();
	
	unsigned int counter;
	for (counter = 0; counter < this->particleArray.size(); counter++)
	{
		meanX = this->particleArray.at(counter).x;
		meanY = this->particleArray.at(counter).y;
		meanTh = this->particleArray.at(counter).th;
	}
	
	LOG(LEVEL_INFO) << "Particle X = " << meanX/particleArray.size();
	LOG(LEVEL_INFO) << "Particle Y = " << meanY/particleArray.size();
	LOG(LEVEL_INFO) << "Particle Th = " << meanTh/particleArray.size();
}
