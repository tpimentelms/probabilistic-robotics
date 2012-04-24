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
		r.setRotVel(2);
		
		// set desired v and w
		move(r.getVel(), r.getRotVel());
		
		// update laser measurement array
		sense();
		
	}
	
	return 0;
}

void move(double v, double w)
{
	double vel, rotVel;
	
	r.setVel(v);
	r.setRotVel(w);
	
	vel = randomGaussianNoise(r.getMoveVelSigma(), v);
	rotVel = randomGaussianNoise(r.getMoveRotVelSigma(), w);
	
	p2dProxy.SetSpeed(vel, rotVel);
}

void sense()
{	
	r.updateLaserArray();
	
	//r.printLaserValue(5);
}

double randomGaussianNoise(double sigma, double mean)
{
	double gaussianNumber, randomNumber1, randomNumber2;
	
	randomNumber1 = double((rand() % 5000))/5000;
	randomNumber2 = double((rand() % 5000))/5000;
	
	while(randomNumber1 == 1 || randomNumber1 == 0)
	{
		randomNumber1 = double((rand() % 5000))/5000;
	}
	while(randomNumber2 == 1 || randomNumber2 == 0)
	{
		randomNumber2 = double((rand() % 5000))/5000;
	}
	
    gaussianNumber = pow(-2*log(randomNumber1),0.5)*cos(2*M_PI*randomNumber2)*sigma+mean;
	
	//LOG(LEVEL_WARN) << "Gaussian info";
	//LOG(LEVEL_INFO) << "Gaussian Number = " << randomNumber1;
	//LOG(LEVEL_INFO) << "Gaussian Number = " << randomNumber2;
	//LOG(LEVEL_INFO) << "Gaussian Number = " << gaussianNumber;
	
	return gaussianNumber;
}
