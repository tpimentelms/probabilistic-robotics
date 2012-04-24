#ifndef ROBOT_H
#define ROBOT_H

#include <ProcessLogger.h>
#include <libplayerc++/playerc++.h>
#include <vector>
#include <Controller.hpp>

extern PlayerCc::Position2dProxy p2dProxy;
extern PlayerCc::LaserProxy laserProxy;

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
		
		double getVelSigma();
		double getRotVelSigma();
		double getLaserSigma();
		
		void setX(double x);
		void setY(double y);
		void setTh(double th);
		void setVel(double v);
		void setRotVel(double w);
		void setPose(double x, double y, double th);
		
		void setVelSigma();
		void setRotVelSigma();
		
		void printInfo();
		void printInfoComparation();
		
		void getTrueState();
		
		
		//Measurements
		std::vector<double> getLaserArray();
		double getLaserValue(unsigned int value);
		
		void printLaserArray();
		void printLaserValue(unsigned int value);
		
		void updateLaserArray();
	
	private:
		double x, y, th;
		double v, w;
		double velSigma, rotVelSigma, laserSigma;
		std::vector<double> laserArray;
};

#endif
