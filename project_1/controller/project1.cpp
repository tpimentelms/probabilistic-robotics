// Date: 16/04/2010
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <libplayerc++/playerc++.h>
#include <Robot.hpp>


void Wander(double *forwardSpeed, double *turnSpeed)
{
        int maxSpeed = 1;
        int maxTurn = 90;
        double fspeed, tspeed;
        
        //fspeed is between 0 and 10
        fspeed = rand()%11;
        //(fspeed/10) is between 0 and 1
        fspeed = (fspeed/10)*maxSpeed;
        
        tspeed = rand()%(2*maxTurn);
        tspeed = tspeed-maxTurn;
        
        *forwardSpeed = fspeed;
        *turnSpeed = tspeed;
}

int main(int argc, char *argv[])
{
		/*need to do this line in c++ only*/
		using namespace PlayerCc;
        
		PlayerClient robot("localhost");
        
        Position2dProxy p2dProxy(&robot,0);
        LaserProxy laserProxy(&robot,0);
        
        //connect to proxies
        double forwardSpeed, turnSpeed;
        srand(time(NULL));
        //enable motors
        //request geometries
        
        while(true)
        {
                // read from the proxies
                robot.Read();
                //wander
                Wander(&forwardSpeed, &turnSpeed);
                
                //set motors
                p2dProxy.SetSpeed(forwardSpeed, dtor(turnSpeed));
                sleep(1);
        }
}
