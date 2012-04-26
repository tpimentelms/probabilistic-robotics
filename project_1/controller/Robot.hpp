#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Controller.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>
#include <armadillo>

using namespace std;
using namespace PlayerCc;
using namespace arma;

extern PlayerClient playerRobot;
extern Position2dProxy p2dProxy;
extern LaserProxy laserProxy;

class Robot
{
	public:
		Robot();
		~Robot();
		
		//Positions and velocities
		double getX();
		double getY();
		double getTh();
		double getVel();
		double getRotVel();
		int    getStrategy();
		
		double getMoveVelSigma();
		double getMoveRotVelSigma();
		
		mat getSigma();

		void setX(double x);
		void setY(double y);
		void setTh(double th);
		void setVel(double v);
		void setRotVel(double w);
		void setPose(double x, double y, double th);
		void setStrategy(int strategy);
		
        void updateState();
        void updateSigma(mat newSigma);
        
		void printInfo();
		void printInfoComparison();
		
		//Measurements
		vector<double> getLaserReadings();
		vector<double> getValidLaserReadings();
		double getOneLaserReading(unsigned int value);
		
		void printLaserReadings();
		void printValidLaserReadings();
		void printOneLaserReading(unsigned int value);
		
		void updateLaserReadings();

	
	private:
		double x, y, th;
		double v, w;
		double moveVelSigma, moveRotVelSigma; // Needs to put this in a matrix, covariance of Ut
		vector<double> laserReadings;
		vector<double> validLaserReadings;
		mat Sigma;
		int strategy;
		//matrix Xbart, Ut, Xt, At, Bt
};

#endif
