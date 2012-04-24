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
		r.updateState();
		
		r.printInfoComparison();
        
        /*
         * TODO: remove this setVel and setRotVel!
         * experimental circle, just to see if everything is OK.
         */
		r.setVel(1);
		r.setRotVel(2);
		
        
		move(r.getVel(), r.getRotVel());
		sense();
		kalmanFilter();
        
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
	r.updateLaserReadings();
	//r.printLaserReadings();
    
    /*
     * TODO: Write interpretation function, which will do some analysis
     * with laser readings and return distances to walls and corners, as
     * well as to the landmark, when found.
     * 
     * interpretMeasurements()
     */
}

mat predictMean(mat A, mat B)
{
	mat ut = createUt();
	mat mu = createMu();
	mat muBar = A*mu + B*ut;
	
	
	LOG(LEVEL_WARN) << "A = " << endl << A;	
	//LOG(LEVEL_WARN) << "mu = " << endl << mu;	
	LOG(LEVEL_WARN) << "B = " << endl << B;	
	//LOG(LEVEL_WARN) << "ut = " << endl << ut;	
    /*
     * TODO: Write functions to build matrices and vectors to be used
     * by the Kalman Filter, such as mean, covariance, control and mea-
     * surements vectors, always using armadillo linear algebra library.
     * 
     *  mean = createMean();
     *  cov = createCov();
     *  ut = createUt();
     *  zt = createZt();
     *  Bt = updateBt();
     */
    
    return muBar;
}

mat createAt()
{
	mat A = eye<mat>(7,7);
	
	A(3, 3) = 0;
	A(4, 4) = 0;
	
	return A;
}

mat createBt()
{
	mat B = zeros<mat>(7,2);
	
	//Velocity parameters
	B(0, 0) = cos(r.getTh());
	B(1, 0) = sin(r.getTh());
	B(2, 0) = 0;
	B(3, 0) = 1;
	B(4, 0) = 0;
	B(5, 0) = 0;
	B(6, 0) = 0;
	
	//Rotation parameters
	B(0, 1) = 0;
	B(1, 1) = 0;
	B(2, 1) = 1;
	B(3, 1) = 0;
	B(4, 1) = 1;
	B(5, 1) = 0;
	B(6, 1) = 0;
	
	return B;

}

/*mat createUt()
{
	mat ut = zeros<mat>(2,1);
	
	ut(0,0) = r.getVel();
	ut(1,0) = r.getRotVel();
	
	return ut;
}

mat createMu()
{
	mat mu = zeros<mat>(7,1);
	
	mu(0,0) = r.getX();
	mu(1,0) = r.getY();
	mu(2,0) = r.getTh();
	mu(3,0) = r.getVel();
	mu(4,0) = r.getRotVel();
	mu(5,0) = 0;
	mu(6,0) = 0;
	
	return mu;
}
*/
void kalmanFilter()
{
	mat muBar = zeros<mat>(7,1);
	mat A = createAt();
	mat B = createBt();
	muBar = predictMean(A, B);
	//predictionMean(A, B)-> returns muBar
	//predictionCov(A)-> returns covBar
	//void postionUpdate(muBar, covBar)
    /*
     * TODO: Implement Kalman Filter algorithm.
     * TODO: Write functions to create each Kalman Filter matrix, such
     * as At, Bt and Ct, using armadillo.
     */
}

void strategy()
{
    /*
     * TODO: Write this function, which will choose the best action to
     * take after analysing all avaiable data.
     */
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
