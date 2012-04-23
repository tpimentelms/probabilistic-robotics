#include <ProcessLogger.h>
#include <libplayerc++/playerc++.h>
#include <vector>

extern PlayerCc::Position2dProxy p2dProxy;
extern PlayerCc::LaserProxy laserProxy;

class Robot
{
	public:
		Robot();
		~Robot();
	
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
	
		void printInfo();
		
		void getTrueState();

		void setPose(double x, double y, double th);

		void updateLaserArray();
		
		std::vector<double> getLaserArray();
		double getLaserValue(unsigned int value);
		void printLaserArray();
		void printLaserValue(unsigned int value);
	
	private:
		double x, y, th;
		double v, w;
		std::vector<double> laserArray;
};
