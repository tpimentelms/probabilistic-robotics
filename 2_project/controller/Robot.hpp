#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Controller.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>
#include <Map.hpp>
#include <Particles.hpp>

using namespace std;
using namespace PlayerCc;
using namespace arma;

extern PlayerClient playerRobot;
extern Position2dProxy p2dProxy;
extern LaserProxy laserProxy;
extern BlobfinderProxy blobProxy;

class Robot
{
	public:
		Robot();
		~Robot();
		
		//Positions and velocities
		double getX();
		double getY();
		double getTh();
		double getLandmarkX();
		double getLandmarkY();
		double getVel();
		double getRotVel();
		int getStrategy();
		
		double getMoveVelSigma();
		double getMoveRotVelSigma();
		
		mat getSigma();
		
		void setX(double x);
		void setY(double y);
		void setTh(double th);
		void setLandmarkX(double landmarkX);
		void setLandmarkY(double landmarkY);
		void setVel(double v);
		void setRotVel(double w);
		void setPose(double x, double y, double th);
		void setStrategy(int strategy);
		
		void updateReadings();
        void updateState(mat newMu);
        void updateSigma(mat newSigma);
        
		void printInfo();
		void printInfoComparison();
		void printSigmaComparison();
		void printRobotParticles();
		void printRobotParticlesMeans();
		
		//Measurements
		vector<double> getLaserReadings();
		vector<int> getValidLaserReadings();
		double getOneLaserReading(unsigned int value);
		double getIfValidLaserReading(unsigned int value);
		
		void printLaserReadings();
		void printValidLaserReadings();
		void printOneLaserReading(unsigned int value);
		
		void updateLaserReadings();
	
	
	private:
		double x, y, th, landmarkX, landmarkY;
		double v, w;
		double moveVelSigma, moveRotVelSigma; // Needs to put this in a matrix, covariance of Ut
		vector<double> laserReadings;
		vector<int> validLaserReadings;
		mat Sigma;
		int strategy;
		Particles robotParticles;
		//matrix Xbart, Ut, Xt, At, Bt
};

#endif
