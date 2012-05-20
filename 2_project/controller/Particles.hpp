#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

using namespace std;
using namespace arma;

#define PARTICLES_SIZE 500

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
		
		vector<particle> getParticles();
		void setParticles(vector<particle> newParticles);
		
	private:
		vector<particle> particleArray;
};

#endif
