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
		
		double getXSigma();
		double getYSigma();
		double getThSigma();
		double getVelSigma();
		double getRotVelSigma();
		double getMoveVelSigma();
		double getMoveRotVelSigma();
		double getLaserSigma();
		
		void setX(double x);
		void setY(double y);
		void setTh(double th);
		void setVel(double v);
		void setRotVel(double w);
		void setPose(double x, double y, double th);
		
		void setXSigma(double xSigma);
		void setYSigma(double ySigma);
		void setThSigma(double thSigma);
		void setVelSigma(double vSigma);
		void setRotVelSigma(double wSigma);
		
		void printInfo();
		void printInfoComparation();
		
		void getTrueState();
		
		
		//Measurements
		std::vector<double> getLaserArray();
		double getLaserValue(unsigned int value);
		
		void printLaserArray();
		void printLaserValue(unsigned int value);
		
		void updateLaserArray();
		void updatePose(double v, double w, double dv, double dw);

	
	private:
		double x, y, th;
		double v, w;
		double velSigma, rotVelSigma;
		double moveVelSigma, moveRotVelSigma, laserSigma;
		std::vector<double> laserArray;
};

#endif
