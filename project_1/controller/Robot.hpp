#include <ProcessLogger.h>

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
	
	private:
		double x, y, th;
		double v, w;
};
