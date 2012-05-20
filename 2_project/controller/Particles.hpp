#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

using namespace std;
using namespace arma;

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
		
	private:
		vector<particle> particleArray;
};

#endif
