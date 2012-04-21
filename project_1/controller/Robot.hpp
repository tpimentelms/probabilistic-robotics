#include <ProcessLogger.h>
#include <libplayerc++/playerc++.h>

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

		
		void laser();

		void setPose(double x, double y, double th);

	
	private:
		double x, y, th;
		double v, w;
};
