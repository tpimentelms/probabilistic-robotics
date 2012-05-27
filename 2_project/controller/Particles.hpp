#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>
#include <cstdio>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  

using namespace std;
using namespace arma;
using namespace PlayerCc;

#define PARTICLES_SIZE 5000

struct particle_t
{
    double x, y, th;
};
typedef struct particle_t particle;

class Particles
{
	public:
		Particles();
		~Particles();
		
		void printParticlesPositions();
		void printParticlesPositionsMeans();
		void showParticlesPositions();
		
		vector<particle> getParticles();
		void setParticles(vector<particle> newParticles);
		
	private:
		vector<particle> particleArray;
};

#endif
