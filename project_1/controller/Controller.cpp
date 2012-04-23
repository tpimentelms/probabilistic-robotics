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
		
		r.printInfoComparation();
		r.setVel(1);
		r.setRotVel(0);
		
		// set desired v and w
		move(r.getVel(), r.getRotVel());
		
		// update laser measurement array
		sense();

	}
	
	return 0;
}

void move(double v, double w)
{
	double velError, rotVelError;
	
	//Make velocity error
	srand(time(NULL));
	velError = ((rand() % (100000))*(2*v/10))/100000;
	velError = velError - v/10;
	
	r.setVel(v + velError);
	
	//Make rotation error
	rotVelError = (rand() % (100000))*(2*w/10)/100000;
	rotVelError = rotVelError - w/10;
	srand(time(NULL));
	
	LOG(LEVEL_WARN) << "velError = " << velError << " rotError = " << rotVelError;
	r.setRotVel(w + rotVelError);
	
	p2dProxy.SetSpeed(r.getVel(), r.getRotVel());
}

void sense()
{
	r.updateLaserArray();
	
	r.printLaserValue(5);
}
