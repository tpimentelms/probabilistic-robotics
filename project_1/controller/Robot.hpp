#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Controller.hpp>
#include <ProcessLogger.h>

#include <libplayerc++/playerc++.h>
#include <vector>

using namespace std;
using namespace PlayerCc;

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
		
		void setX(double x);
		void setY(double y);
		void setTh(double th);
		void setVel(double v);
		void setRotVel(double w);
		void setPose(double x, double y, double th);
		
        void updateState();
        
		void printInfo();
		void printInfoComparison();
		
		//Measurements
		vector<double> getLaserReadings();
		double getOneLaserReading(unsigned int value);
		
		void printLaserReadings();
		void printOneLaserReading(unsigned int value);
		
		void updateLaserReadings();

	
	private:
		double x, y, th;
		double v, w;
		vector<double> laserReadings;
};

#endif
