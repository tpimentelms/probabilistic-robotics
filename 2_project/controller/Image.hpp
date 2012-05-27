#ifndef IMAGE_HPP
#define IMAGE_HPP


#include <Robot.hpp>
#include <Landmark.hpp>
#include <Particles.hpp>
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

class Image
{
	public:
		Image();
		~Image();
		
		void showParticlesPositions(Robot r, Landmark l);
		
	private:
		IplImage *particlesImage;
		IplImage *originalImage;
};

#endif
