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
	//r.printValidLaserReadings();
    
    /*
     * TODO: Write interpretation function, which will do some analysis
     * with laser readings and return distances to walls and corners, as
     * well as to the landmark, when found.
     * 
     * interpretMeasurements()
     */
     
     interpretMeasurements();
}

bool interpretMeasurements()
{
	bool foundSomething;
	
	if (findLine() || findLandmark())
	{
		foundSomething = 1;
		if (findLine() == 2)
		{
			findCorner();
		}
		return 1;
	}
	
	
	return 0;
}

int findLine()
{
	unsigned int i, j, k;
	int number, counter;
	double deltaX, deltaY;
	vector<double> laserMeasurements = r.getLaserReadings();
	vector<double> validLaserMeasurements = r.getValidLaserReadings();
	vector<double> cosOfLine;
	mat houghMatrix = zeros<mat>(validLaserMeasurements.size(), 1000);
	
	
	LOG(LEVEL_WARN) << "Finding Line Info";
	/*
	for (i=0;i<1000;i++)
	{
		for(j=0; j<laserMeasurements.size(); j++)
		{
			deltaX = laserMeasurements.at(j)*cos(r.getTh()+dtor(j));
			deltaY = laserMeasurements.at(j)*sin(r.getTh()+dtor(j));
			houghMatrix(j, i) = deltaX*cos(i*M_PI/1000) + deltaY*sin(i*M_PI/1000);
		}
	}
	*/
	
	for (i=0;i<1000;i++)
	{
		for(j=0; j<validLaserMeasurements.size(); j++)
		{
			deltaX = laserMeasurements.at(validLaserMeasurements.at(j))*cos(r.getTh()+dtor((j+180)%360));
			deltaY = laserMeasurements.at(validLaserMeasurements.at(j))*sin(r.getTh()+dtor((j+180)%360));
			houghMatrix(j, i) = deltaX*cos(i*M_PI/1000) + deltaY*sin(i*M_PI/1000);
		}
	}
	
	
	for(j=0; j < validLaserMeasurements.size(); j++)
	{
		number = 0;
		for (i=0;i<1000;i++)
		{
			counter = 0;
			for (k=0; k<validLaserMeasurements.size(); k++)
			{
				if (0.05 > abs(houghMatrix(j, i) - houghMatrix(k, i)))
				{
					counter = counter + 1;
					//cosOfLine.push_back(i); 	//create something that locks the thetas so we can them compare and,
												// if it's really different, consider two diferent lines.
				}
			}
			if(counter > 50)
				number = number + 1;
		}
		if (number>0)
		{
			LOG(LEVEL_INFO) << "Number[" << validLaserMeasurements.at(j) << "] = " << number;
			LOG(LEVEL_INFO) << "Theta[" << validLaserMeasurements.at(j) << "] = " << r.getTh();
			for ()
		}
	}
	
	
	return 0;
}

bool findCorner()
{
	return 0;
}

bool findLandmark()
{
	return 0;
}

mat predictMean(mat A, mat B)
{
	mat ut = createUt();
	mat mu = createMu();
	mat muBar = A*mu + B*ut;
	
	
	
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

mat predictiCov(mat A)
{
	mat R = createRt();
	mat sigma = r.getSigma();
	mat sigmaBar = A * sigma * A.t() + R;
	
	return sigmaBar;
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

mat createUt()
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

mat createRt()
{
	mat R = eye<mat>(7,7);
	double theta = r.getTh();
	R(0,0) = r.getMoveVelSigma() * abs(cos(theta));
	R(1,1) = r.getMoveVelSigma() * abs(sin(theta));
	R(2,2) = r.getMoveRotVelSigma();
	R(3,3) = r.getMoveVelSigma();
	R(4,4) = r.getMoveRotVelSigma();
	R(5,5) = 0;
	R(6,6) = 0;
	
	return R;
}


void kalmanFilter()
{
	mat A = createAt();
	mat B = createBt();
	mat muBar = predictMean(A, B);
	mat sigmaBar = predictiCov(A);
	r.updateSigma(sigmaBar);
	LOG(LEVEL_WARN) << "Sigma = " << endl << sigmaBar;
	//
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
