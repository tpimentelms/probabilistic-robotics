#include <Controller.hpp>

PlayerClient playerRobot("localhost");
Position2dProxy p2dProxy(&playerRobot, 0);
LaserProxy laserProxy(&playerRobot, 0);

Map worldMap, robotMap;

Robot r;

int main()
{
	vector<wallsFound> lines;
    laserProxy.RequestGeom();
	
	worldMap.createKnownMap();
	
    while(true)
    {
		//r.updateState();
        
        /*
         * TODO: remove this setVel and setRotVel!
         * experimental circle, just to see if everything is OK.
         */
        lines = sense();
         
   		//strategy(lines);

		move(r.getVel(), r.getRotVel());

		kalmanFilter(lines);
		
		r.printInfoComparison();
		r.printSigmaComparison();
	
	}
	
	return 0;
}

void move(double v, double w)
{
	if(w>M_PI_4)
		w=M_PI_4;
	
	r.setVel(v);
	r.setRotVel(w);
	
	if(v!=0 || w != 0)
	{
		v = randomGaussianNoise(r.getMoveVelSigma()*v, v);
		w = randomGaussianNoise(r.getMoveRotVelSigma()*w, w);
	}
	w = 0.1;
	r.setRotVel(w);

	p2dProxy.SetSpeed(0, w);
}

vector<wallsFound> sense()
{
	vector<wallsFound> lines;
	
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
     
     lines = interpretMeasurements();
     return lines;
}

vector<wallsFound> interpretMeasurements()
{
	vector<wallsFound> lines;
	point corner;
	point landmark;
	pair<bool, point> returnedCorner;
	pair<bool, point> returnedLandmark;
	
	lines = findLine();
	returnedLandmark = findLandmark();//make_pair(0, landmark);
	
	//checks if something was found
	if (lines.size() || returnedLandmark.first)
	{
		//found only a line
		if (lines.size() == 1)
		{
/*			LOG(LEVEL_WARN) << "Line found";
			LOG(LEVEL_INFO) << "Distance = " << lines.at(0).distance;
			LOG(LEVEL_INFO) << "Theta = " << lines.at(0).angle;
*/		}
		//landmark found
		if (returnedLandmark.first)
		{
			landmark = returnedLandmark.second;
			updateLandmarkState(landmark);
/*			LOG(LEVEL_WARN) << "Landmark Found";
			LOG(LEVEL_INFO) << "Landmark X = " << landmark.x;
			LOG(LEVEL_INFO) << "Landmark Y = " << landmark.y;
*/		}
		//found a corner
		if (lines.size() == 2)
		{
			returnedCorner = findCorner(lines);
			corner = returnedCorner.second;
			
			if(returnedCorner.first == 1)
				LOG(LEVEL_ERROR) << "Open Corner Found";
			else
				LOG(LEVEL_ERROR) << "Closed Corner Found";
/*			
			LOG(LEVEL_INFO) << "Corner X = " << corner.x;
			LOG(LEVEL_INFO) << "Corner Y = " << corner.y;
*/		}
	}
	
	
	return lines;
}

vector<wallsFound> findLine()
{
	//variable declarations
	unsigned int i, j, k;
	int number, counter;
	double deltaX, deltaY;
	vector<double> laserMeasurements = r.getLaserReadings();
	vector<int> validLaserMeasurements = r.getValidLaserReadings();
	vector<double> cosOfLine, cosMeans, distanceOfLine, distanceMeans;
	vector<double> getPositions;
	vector<wallsFound> lines;
	mat houghMatrix = zeros<mat>(validLaserMeasurements.size(), 1000);
	
	//empties vectors used
	cosMeans.clear();
	getPositions.clear();
	
	//transforms measurements distances to hough space
	for (i=0;i<1000;i++)
	{
		for(j=0; j<validLaserMeasurements.size(); j++)
		{
			deltaX = laserMeasurements.at(validLaserMeasurements.at(j))*cos(dtor((validLaserMeasurements.at(j)+180)%360));
			deltaY = laserMeasurements.at(validLaserMeasurements.at(j))*sin(dtor((validLaserMeasurements.at(j)+180)%360));
			houghMatrix(j, i) = deltaX*cos(i*M_PI/1000) + deltaY*sin(i*M_PI/1000);
		}
	}
	
	//checks which of them is part of a line
	for(j=0; j < validLaserMeasurements.size(); j++)
	{
		number = 0;
		cosOfLine.clear();
		distanceOfLine.clear();
		for (i=0;i<1000;i++)
		{
			counter = 0;
			for (k=0; k<=60 && k <= validLaserMeasurements.size(); k++)
			{
				//counts how many points form a line with another point
				if (0.08 > abs(houghMatrix(j, i) - houghMatrix((j+k-30)%validLaserMeasurements.size(), i)) && k != 30)
				{
					counter = counter + 1;
				}
			}
			if(counter > 40)
			{
				// gets angles and distances for possible lines and counts how many realy possible different lines pass trough a point
				number = number + 1;
				cosOfLine.push_back(i);
				distanceOfLine.push_back(abs(houghMatrix(j, i)));
			}
		}
		if (number>0)
		{
			//get means of all angles and distances with possible lines from each point
			cosMeans.push_back(getMeanRoundWorld(cosOfLine, 1000));
			distanceMeans.push_back(getMean(distanceOfLine));
			getPositions.push_back(validLaserMeasurements.at(j));
		}
	}
	
	//checks if theres any line
	if (cosMeans.size() == 0)
	{
		return lines;
	}
	
	//gets lines in a vector and returns them
	lines = getLines(cosMeans, getPositions, laserMeasurements, distanceMeans);
	
	return lines;
}

pair<bool, point> findCorner(vector<wallsFound> lines)
{
	double a, b, c, d;
	int theta;
	double valid1, valid2;
	double valid3, valid4;
	point corner;
	
	//parameters of both lines
	a = -cos(lines.at(0).angle)/sin(lines.at(0).angle);
	b = lines.at(0).distance/sin(lines.at(0).angle);
	c = -cos(lines.at(1).angle)/sin(lines.at(1).angle);
	d = lines.at(1).distance/sin(lines.at(1).angle);
	
	//find interssection of lines
	corner.y = (d-b)/(a-c);
	corner.x = -(a*corner.y + b);
	
	theta = int(rtod(atan2(corner.y, corner.x))) + 90;
	if(theta < 0)
	{
		theta += 360;
	}
	
	valid1 = r.getIfValidLaserReading((theta+60)%360);
	valid2 = r.getIfValidLaserReading((theta-60+360)%360);
	valid4 = r.getIfValidLaserReading((theta+90)%360);
	valid3 = r.getIfValidLaserReading((theta-90+360)%360);
	if ((valid1 == -1 && valid3 == -1) || (valid2 == -1 && valid4 == -1))
	{
		return make_pair(1, corner);
		
	}
	else
	{
		return make_pair(0, corner);
	}
		
	
}

pair<bool, point> findLandmark()
{
	point landmark;
	int i, first, last;
	double distanceToLandmark, sumOfDistances, angle;
	vector<double> laserMeasurements = r.getLaserReadings();
	vector<int> validLaserMeasurements = r.getValidLaserReadings();
	
	pair<int, int> sensorsUsed = findLandmarkClusterOfMeasures(validLaserMeasurements, laserMeasurements);
	
	if(sensorsUsed.first == 0)
	{
		landmark.x = 0;
		landmark.y = 0;
		return make_pair(0, landmark);
	}
	
	first = sensorsUsed.first;
	last = sensorsUsed.second;
	
	sumOfDistances = 0;
		
	for (i = first; i <= last ; i++)
	{
		sumOfDistances += abs(cos(dtor(abs(i-(last+first)/2)))*laserMeasurements.at(i))+RAIO;
	}
	
	distanceToLandmark = sumOfDistances/(last-first+1);
	
	
	angle = dtor((int((last+first)/2) + 270) % 360);
	
	landmark.x = cos(angle)*distanceToLandmark;
	landmark.y = sin(angle)*distanceToLandmark;
	
	return make_pair(1, landmark);
}

void updateLandmarkState(point landmark)
{
	double landmarkX = sin(r.getTh())*landmark.x + cos(r.getTh())*landmark.y + r.getX();
	double landmarkY = -cos(r.getTh())*landmark.x + sin(r.getTh())*landmark.y + r.getY();
	
	LOG(LEVEL_INFO) << "Landmark X = " << landmarkX;
	LOG(LEVEL_INFO) << "Landmark Y = " << landmarkY;
	
	
	mat muBar = createMu();
	mat sigmaBar = r.getSigma();
	
	mat C = createCtLandmark();
	mat Q = createQtLandmark();
	mat Z = createZtLandmark(landmarkX, landmarkY);
	
	mat K = sigmaBar*C.t()*(C*sigmaBar*C.t() + Q).i();
	muBar = muBar + K*(Z-C*muBar);
	sigmaBar = (eye<mat>(7,7)-K*C)*sigmaBar;
	
	r.updateState(muBar);
	r.updateSigma(sigmaBar);
}

mat predictMean(mat A, mat B)
{
	mat ut = createUt();
	mat mu = createMu();
	mat muBar = A*mu + B*ut;
	LOG(LEVEL_WARN) << "muBar(2,0) = " << muBar(2,0)*(360/(2*M_PI));
	
	
	
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

mat predictCov(mat A)
{
	mat R = createRt();
	mat sigma = r.getSigma();
	mat sigmaBar = A * sigma * trans(A) + R;
	
	return sigmaBar;
}


mat createAt()
{
	mat A = eye<mat>(7,7);
	
	A(3, 3) = 0;
	A(4, 4) = 0;
	
	return A;
}

mat createBt(double deltaT)
{
	mat B = zeros<mat>(7,2);
	
	//Velocity parameters
	B(0, 0) = cos(r.getTh())*deltaT;
	B(1, 0) = sin(r.getTh())*deltaT;
	B(2, 0) = 0;
	B(3, 0) = 1;
	B(4, 0) = 0;
	B(5, 0) = 0;
	B(6, 0) = 0;
	
	//Rotation parameters
	B(0, 1) = 0;
	B(1, 1) = 0;
	B(2, 1) = 1*deltaT;
	B(3, 1) = 0;
	B(4, 1) = 1;
	B(5, 1) = 0;
	B(6, 1) = 0;
	
	return B;

}

mat createCt()
{
	mat C = zeros<mat>(3,7);
	
	C(0, 0) = 1;
	C(1, 1) = 1;
	C(2, 2) = 1;
	
	return C;
}

mat createCtLandmark()
{
	mat C = zeros<mat>(2,7);
	
	C(0, 5) = 1;
	C(1, 6) = 1;
	
	return C;
	
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
	mu(5,0) = r.getLandmarkX();
	mu(6,0) = r.getLandmarkY();
	
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

mat createQt()
{
	mat Q = zeros<mat>(3,3);
	
	//Covariance of measurements
	Q(0, 0) = 0.2;
	Q(1, 1) = 0.2;
	Q(2, 2) = 0.2;
	
	return Q;
}

mat createQtLandmark()
{
	mat Q = zeros<mat>(2,2);
	double xSigma = r.getSigma()(0,0);
	double ySigma = r.getSigma()(1,1);
	double thetaSigma = r.getSigma()(2,2);
	
	//Covariance of measurements
	Q(0, 0) = 0.01 + xSigma + thetaSigma;
	Q(1, 1) = 0.01 + ySigma + thetaSigma;
	
	return Q;
}

mat createZt(double robotX, double robotY, double robotTheta)
{
	mat Z = zeros<mat>(3,1);
	
	//Measurements
	Z(0, 0) = robotX;
	Z(1, 0) = robotY;
	Z(2, 0) = robotTheta;
	
	return Z;
}

mat createZtLandmark(double landmarkX, double landmarkY)
{
	mat Z = zeros<mat>(2,1);
	
	//Measurements
	Z(0, 0) = landmarkX;
	Z(1, 0) = landmarkY;
	
	return Z;
}


void kalmanFilter(vector<wallsFound> lines)
{
	static struct timeval earlier;
	struct timeval later;
	double theta1, theta2, robotX, robotY, robotTheta;
	double deltaT;
	
	gettimeofday(&later,NULL);
	if (earlier.tv_usec == 0 && earlier.tv_sec == 0)
		deltaT = 1;
	else
		deltaT = double(timeval_diff(NULL,&later,&earlier))/1000000;
	
	//deltaT = 1;
	LOG(LEVEL_WARN) << "Time found = " << deltaT;
	
	
	
	mat A = createAt();
	mat B = createBt(deltaT);
	mat muBar = predictMean(A, B);
	mat sigmaBar = predictCov(A);
	r.updateSigma(sigmaBar);
	
	
	if(lines.size() == 2)
	{
		pair<bool, point> corner = findCorner(lines);
		//only does measurement update if seeing the corner. needs something to be oriented from.
		if (corner.first)
		{
			point a = corner.second;
			theta1 = lines.at(0).angle;
			theta2 = lines.at(1).angle;
			
			if(((theta1 - theta2) > 0 && (theta1 - theta2) < M_PI) || (theta1 - theta2) < -M_PI)
			{
				robotX = lines.at(0).distance;
				robotY = -lines.at(1).distance;
				robotTheta = dtor(int(rtod(5*M_PI/2 - theta2)) % 360 + 25);
			}
			else
			{
				robotY = -lines.at(0).distance;
				robotX = lines.at(1).distance;
				robotTheta = dtor(int(rtod(5*M_PI/2 - theta1)) % 360 + 25);
			}
			mat C = createCt();
			mat Q = createQt();
			mat Z = createZt(robotX, robotY, robotTheta);
			
			mat K = sigmaBar*C.t()*(C*sigmaBar*C.t() + Q).i();
			muBar = muBar + K*(Z-C*muBar);
			sigmaBar = (eye<mat>(7,7)-K*C)*sigmaBar;
			
			/*
			LOG(LEVEL_WARN) << "Open Corner Found";
			LOG(LEVEL_WARN) << "Theta 2 = " << theta2;
			LOG(LEVEL_WARN) << "Theta 1 = " << theta1;
			LOG(LEVEL_WARN) << "Robot Y = " << lines.at(0).distance;
			LOG(LEVEL_WARN) << "Robot Theta = " << lines.at(1).distance;
			LOG(LEVEL_WARN) << "Robot X = " << robotX;
			LOG(LEVEL_WARN) << "Robot Y = " << robotY;
			LOG(LEVEL_WARN) << "Robot Theta = " << robotTheta << "Real Theta" << p2dProxy.GetYaw();
			*/
			
		}
	}
	gettimeofday(&earlier,NULL);
	r.updateState(muBar);
	r.updateSigma(sigmaBar);
	
	//
	//void postionUpdate(muBar, covBar)
    /*
     * TODO: Implement Kalman Filter algorithm.
     * TODO: Write functions to create each Kalman Filter matrix, such
     * as At, Bt and Ct, using armadillo.
     */
}

void strategy(vector<wallsFound> lines)
{
	int strategy_state = r.getStrategy();
	static bool wallFound = false, twoWalls = false;
	
	switch(strategy_state)
	{
		case 1://Estratégia de busca do canto
			
//			bool detectObject = interpretMeasurements();
			vector<wallsFound> lines = findLine();
			
			if(lines.size() >= 1)
			{
				if(lines.size() > 1 && wallFound && !twoWalls)
				{
					wallFound = false;
					twoWalls = true;
				}
				if (twoWalls && wallFound)
				{
					twoWalls = false;
				}
				vector<wallsFound> lines = findLine();
				followWall(lines, wallFound);
				wallFound = true;

			}
			else
			{
				r.setVel(0.3);
				r.setRotVel(0);
				if(wallFound == true)
				{
					r.setRotVel(-0.3);
				}
			}
			
		break;
	}
}

void followWall(vector<wallsFound> lines, int wallFound)
{
	//double rotation;
	int i = 0;
	
	double param = 0.1;
	double param2 = 5;
	
	double rotVel;
	
	static double CTE;
	double diffCTE;
	double theta1, theta2;
	
	int line = 1.5;
	
	if(lines.size() == 2)
	{
		if(findCorner(lines).first)
		{
			theta1 = lines.at(0).angle;
			theta2 = lines.at(1).angle;
		}
		else
		{
			theta2 = lines.at(0).angle;
			theta1 = lines.at(1).angle;
		}
		
		if(((theta1 - theta2) > 0 && (theta1 - theta2) < M_PI) || (theta1 - theta2) < -M_PI)
		{
			i = 1;
			line = 1.50;
		}
		
		
	}
	
	if(!wallFound)
	{
		CTE = lines[i].distance - line;
	}
	
	diffCTE = lines[i].distance - line - CTE;
	CTE = lines[i].distance - line;
	r.setVel(0.2);
	rotVel = -param*CTE - param2*diffCTE;
	r.setRotVel(rotVel);
	
	return;
	
}
				
    /*
     * TODO: Write this function, which will choose the best action to
     * take after analysing all avaiable data.
     */


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

double getMeanRoundWorld(vector<double> array, int worldSize)
{
	unsigned int counter;
	double mean1 = 0, mean2 = 0;
	double diff1 = 0, diff2 = 0;
		
	for (counter = 0; counter < array.size(); counter++)
	{
		if (array.at(counter) > worldSize/2)
			mean2 = mean2 + array.at(counter) - worldSize;
		else
			mean2 = mean2 + array.at(counter);
			
		mean1 = mean1 + array.at(counter);
	}
	
	mean1 = mean1 / array.size();
	mean2 = mean2 / array.size();
	
	for (counter = 0; counter < array.size(); counter++)
	{
		if (array.at(counter) > worldSize/2)
			diff2 = diff2 + abs (mean2 - array.at(counter) + worldSize);
		else
			diff2 = diff2 + abs (mean2 - array.at(counter));
		
		diff1 = diff1 + abs (mean1 - array.at(counter));
	}
	
	if(diff1 <= diff2)
		return mean1;
	else
	{
		if (mean2<0)
			return mean2+worldSize;
		else
			return mean2;
	}
}

double getMean(vector<double> array)
{
	unsigned int counter;
	double mean = 0;
		
	for (counter = 0; counter < array.size(); counter++)
	{
		mean = mean + array.at(counter);
	}
	
	mean = mean / array.size();
	return mean;
}

double getBetterAngle (unsigned int sensorUsed, double lineTheta)
{
	double whereMeasurement = (dtor((sensorUsed+180)%360));
	double diff1, diff2;
	
	diff1 = abs(whereMeasurement - lineTheta);
	diff2 = abs(whereMeasurement - lineTheta - M_PI);
	
	if (diff1 > M_PI)
		diff1 = 2 * M_PI - diff1;
	if (diff2 > M_PI)
		diff2 = 2 * M_PI - diff2;
	
	if (diff1 < diff2)
		return lineTheta;
	else
		return lineTheta + M_PI;
}

vector<wallsFound> getLines(vector<double> cosMeans, vector<double> getPositions, vector<double> laserMeasurements, vector<double> distanceMeans)
{
	unsigned int i, k, j;
	bool differentThanAll;
	double lineTheta, lineDistance;
	unsigned int sensorUsed;
	wallsFound singleLine;
	vector<double> differentCosMeans;
	vector<double> passingArguments;
	vector<vector <double> > cosMeansGroups, getPositionsGroups, distanceMeansGroups;
	vector<wallsFound> lines;
		
	passingArguments.clear();
	
	//splitting information in groups related to the different lines
	//creating first group of lines
	differentCosMeans.push_back(cosMeans.at(0));
	
	//initializing new groups
	cosMeansGroups.push_back(passingArguments);
	getPositionsGroups.push_back(passingArguments);
	distanceMeansGroups.push_back(passingArguments);
	
	//passing new arguments
	cosMeansGroups.at(0).push_back(cosMeans.at(0));
	getPositionsGroups.at(0).push_back(getPositions.at(0));
	distanceMeansGroups.at(0).push_back(distanceMeans.at(0));
	
	for (k=0; k<cosMeans.size(); k++)
	{
		differentThanAll = 1;
		for (j=0; j<differentCosMeans.size(); j++)
		{
			if (200 > abs(differentCosMeans.at(j) - cosMeans.at(k)) || 800 < abs(differentCosMeans.at(j) - cosMeans.at(k)))
			{
				//adding new information to existing groups of lines
				differentThanAll = 0;
				cosMeansGroups.at(j).push_back(cosMeans.at(k));
				getPositionsGroups.at(j).push_back(getPositions.at(k));
				distanceMeansGroups.at(j).push_back(distanceMeans.at(k));
			}
			if(differentThanAll == 1 && j == differentCosMeans.size() - 1)
			{
				differentCosMeans.push_back(cosMeans.at(k));
				
				//creating new groups of lines
				cosMeansGroups.push_back(passingArguments);
				getPositionsGroups.push_back(passingArguments);
				distanceMeansGroups.push_back(passingArguments);
				
				//passing new arguments
				cosMeansGroups.at(j+1).push_back(cosMeans.at(k));
				getPositionsGroups.at(j+1).push_back(getPositions.at(k));
				distanceMeansGroups.at(j+1).push_back(distanceMeans.at(k));
			}
		}
	}
	
	//getting information of every line given their already collected information
	for (i=0; i<cosMeansGroups.size(); i++)
	{
		//gets mean of all possible theta, from 0-1000, and which would be the most trustable sensor
		lineTheta = getMeanRoundWorld(cosMeansGroups.at(i), 1000);
		sensorUsed = (unsigned int)(getMeanRoundWorld(getPositionsGroups.at(i), laserMeasurements.size()));
		
		//gets distance to wall
		lineDistance = getMean(distanceMeansGroups.at(i));
				
		//gets theta from 0-2*pi
		lineTheta = lineTheta*M_PI/1000;
		lineTheta = getBetterAngle(sensorUsed, lineTheta);
		
		//passes arguments to vector returned
		singleLine.distance = lineDistance;
		singleLine.angle = lineTheta;
		lines.push_back(singleLine);
	}
	
	return lines;
}

pair<int, int> findLandmarkClusterOfMeasures(vector<int> validLaserMeasurements, vector<double> laserMeasurements)
{
	unsigned int i, counter;
	int last, first;
	double distanceToLandmark, correctAngle;
	
	last = 0;
	counter = 0;
	first = 0;
	
	for(i=0;i<validLaserMeasurements.size();i++)
	{
		if(last == validLaserMeasurements.at(i) - 1)
		{
			last = validLaserMeasurements.at(i);
			counter++;
		}
		else
		{
			distanceToLandmark = laserMeasurements.at(int((last+first)/2))+RAIO;
			correctAngle = rtod(2*atan(RAIO/(distanceToLandmark)));
						
			if(counter < correctAngle + 20 && counter > correctAngle-20 && counter > 25)
			{
				return make_pair(first, last);
			}
			else
			{
				first = validLaserMeasurements.at(i);
				last = validLaserMeasurements.at(i);
				counter = 0;
			}
		}
	}
	
	return make_pair(0, 0);
}

long long int timeval_diff(struct timeval *difference, struct timeval *end_time, struct timeval *start_time)
{
  struct timeval temp_diff;

  if(difference==NULL)
  {
    difference=&temp_diff;
  }

  difference->tv_sec =end_time->tv_sec -start_time->tv_sec ;
  difference->tv_usec=end_time->tv_usec-start_time->tv_usec;

  /* Using while instead of if below makes the code slightly more robust. */

  while(difference->tv_usec<0)
  {
    difference->tv_usec+=1000000;
    difference->tv_sec -=1;
  }

  return 1000000LL*difference->tv_sec+
                   difference->tv_usec;
}
