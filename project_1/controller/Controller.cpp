/** TODO: Transformar movimento em randômico
 *
 */

#include <Controller.hpp>

PlayerClient playerRobot("localhost");
Position2dProxy p2dProxy(&playerRobot, 0);
LaserProxy laserProxy(&playerRobot, 0);

Robot r;

int main()
{
    laserProxy.RequestGeom();

    while(true)
    {
		playerRobot.Read();
		
		// get current pose
		r.getTrueState();
		
		r.printInfo();
		r.setVel(1);
		r.setRotVel(2);
		
		// set desired v and w
		p2dProxy.SetSpeed(0, 0);
		
		//p2dProxy.SetSpeed(r.getVel(), r.getRotVel());
		move(r.getVel(), r.getRotVel());
		sense();

	}
	
	return 0;
}

void move(double v, double w)
{
	r.setVel(v);
	r.setRotVel(w);
	p2dProxy.SetSpeed(r.getVel(), r.getRotVel());
}

void sense()
{
	r.updateLaserArray();
	
	r.printLaserValue(5);
}
