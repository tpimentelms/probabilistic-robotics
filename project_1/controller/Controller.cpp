#include <Controller.hpp>

PlayerClient playerRobot("localhost");
Robot r;

int main()
{
	Position2dProxy p2dProxy(&playerRobot, 0);
    LaserProxy laserProxy(&playerRobot, 0);
    
    while(true)
    {
		playerRobot.Read();
		

		
		p2dProxy.SetSpeed(2, 0);
	}
	
	return 0;
}
