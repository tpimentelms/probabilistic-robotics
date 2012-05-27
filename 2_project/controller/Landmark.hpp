#ifndef LANDMARK_HPP
#define LANDMARK_HPP

#include <ProcessLogger.h>
#include <Particles.hpp>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

using namespace std;
using namespace arma;

class Landmark
{
	public:
		Landmark();
		~Landmark();
		
		void printLandmarkParticlesPositions();
		void printLandmarkPosition();
		
	private:
		Particles landmarkParticles;
		int color;
};

#endif
