#include <Controller.hpp>

PlayerClient playerRobot("localhost");
Position2dProxy p2dProxy(&playerRobot, 0);
LaserProxy laserProxy(&playerRobot, 0);

Robot r;

int main()
{
    while(true)
    {
		playerRobot.Read();
		
		// get current pose
		r.getTrueState();
		
		r.printInfo();
		r.setVel(1);
		r.setRotVel(2);
		
		// set desired v and w
		p2dProxy.SetSpeed(r.getVel(), r.getRotVel());
	}
	
	return 0;
}
