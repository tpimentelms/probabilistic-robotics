#include "Robot.hpp"

Robot::Robot()
{
	this->moveVelSigma = 0.1;
	this->moveRotVelSigma = 0.1;
	this->Sigma = 1000*eye<mat>(7,7);
	this->strategy = 1;
}

Robot::~Robot()
{
}

double Robot::getX()
{
	return this->x;
}

double Robot::getY()
{
	return this->y;
}

double Robot::getTh()
{
	return this->th;
}

double Robot::getLandmarkX()
{
	return this->landmarkX;
}

double Robot::getLandmarkY()
{
	return this->landmarkY;
}

double Robot::getVel()
{
	return this->v;
}

double Robot::getRotVel()
{
	return this->w;
}

double Robot::getMoveVelSigma()
{
	if(this->v == 0 && this->w == 0)
		return 0;
	return this->moveVelSigma;
}

double Robot::getMoveRotVelSigma()
{
	if(this->v == 0 && this->w == 0)
		return 0;
	return this->moveRotVelSigma;
}

mat Robot::getSigma()
{
	return this->Sigma;
}

int Robot::getStrategy()
{
	return this->strategy;
}

void Robot::setX(double x)
{
	this->x = x;
}

void Robot::setY(double y)
{
	this->y = y;
}

void Robot::setTh(double th)
{
	this->th = th;
}

void Robot::setLandmarkX(double landmarkX)
{
	this->landmarkX = landmarkX;
}

void Robot::setLandmarkY(double landmarkY)
{
	this->landmarkY = landmarkY;
}

void Robot::setVel(double v)
{
	this->v = v;
}

void Robot::setRotVel(double w)
{
	this->w = w;
}

void Robot::setPose(double x, double y, double th)
{
	this->setX(x);
	this->setY(y);
	this->setTh(th);
}

void Robot::setStrategy(int strategy)
{
	this->strategy = strategy;
}

void Robot::updateState(mat newMu)
{
    // read current state from player client
    playerRobot.Read();
    
    // cant use gps, this is wrong
	this->setX(newMu(0,0));
	this->setY(newMu(1,0));
	this->setTh(newMu(2,0));
	this->setVel(newMu(3,0));//  No errors present in this measurements, so don't use it
	this->setRotVel(newMu(4,0));
	this->setLandmarkX(newMu(5,0));
	this->setLandmarkY(newMu(6,0));
}

void Robot::updateSigma(mat newSigma)
{
	this->Sigma = newSigma;
}

void Robot::printInfo()
{
	LOG(LEVEL_WARN) << "Robot info";
	LOG(LEVEL_INFO) << "X = " << this->getX();
	LOG(LEVEL_INFO) << "Y = " << this->getY();
	LOG(LEVEL_INFO) << "Th = " << rtod(this->getTh());
	LOG(LEVEL_INFO) << "Vel = " << this->getVel();
	LOG(LEVEL_INFO) << "RotVel = " << this->getRotVel();
}

void Robot::printInfoComparison()
{
	LOG(LEVEL_WARN) << "Robot comparison info";
	LOG(LEVEL_INFO) << "X = " << this->getX() << "\t True X = " << p2dProxy.GetXPos() + 5 << "\t Delta X = " << this->getX()-p2dProxy.GetXPos() + 5;
	LOG(LEVEL_INFO) << "Y = " << this->getY() << "\t True Y = " << p2dProxy.GetYPos() - 3 << "\t Delta Y = " << this->getY()-p2dProxy.GetYPos() - 3;
	LOG(LEVEL_INFO) << "Th = " << rtod(this->getTh()) << "\t True Th = " << rtod(p2dProxy.GetYaw()) << "\t Delta Th = " << rtod(this->getTh()-p2dProxy.GetYaw());
	LOG(LEVEL_INFO) << "Vel = " << this->getVel() << "\t True Vel = " << p2dProxy.GetXSpeed() << "\t Delta Vel = " << this->getVel()-p2dProxy.GetXSpeed();
	LOG(LEVEL_INFO) << "RotVel = " << this->getRotVel() << " True RotVel = " << p2dProxy.GetYawSpeed() << "\t Delta RotVel = " << this->getRotVel()-p2dProxy.GetYawSpeed();
	LOG(LEVEL_INFO) << "Landmark X = " << this->getLandmarkX() << " True Landmark X = " << -3.8 << "\t Delta Landmark X = " << this->getLandmarkX()-3.5;
	LOG(LEVEL_INFO) << "Landmark Y = " << this->getLandmarkY() << " True Landmark Y = " << -3.5 << "\t Delta Landmark Y = " << this->getLandmarkY()+5;
}

void Robot::printSigmaComparison()
{
	LOG(LEVEL_WARN) << "Robot Sigma info";
	LOG(LEVEL_INFO) << "X Sigma = " << this->Sigma(0,0);
	LOG(LEVEL_INFO) << "Y Sigma = " << this->Sigma(1,1);
	LOG(LEVEL_INFO) << "Th Sigma = " << this->Sigma(2,2);
	LOG(LEVEL_INFO) << "Vel Sigma = " << this->Sigma(3,3);
	LOG(LEVEL_INFO) << "RotVel Sigma = " << this->Sigma(4,4);
	LOG(LEVEL_INFO) << "Landmark X Sigma = " << this->Sigma(5,5);
	LOG(LEVEL_INFO) << "Landmark Y Sigma = " << this->Sigma(6,6);
}

void Robot::updateLaserReadings()
{
	unsigned int counter;
	double laserMeasurement;
	
	this->laserReadings.clear();
	this->validLaserReadings.clear();
	
	for (counter = 0; counter < laserProxy.GetCount(); counter++)
	{
		laserMeasurement = laserProxy.GetRange(counter);
		this->laserReadings.push_back(laserMeasurement);
		if (laserMeasurement < 3.7)
		{
			this->validLaserReadings.push_back(counter);
		}
	}
}

vector<double> Robot::getLaserReadings()
{
	return this->laserReadings;
}

vector<int> Robot::getValidLaserReadings()
{
	return this->validLaserReadings;
}

double Robot::getOneLaserReading(unsigned int value)
{
	if (this->laserReadings.size() < value)
	{
		LOG(LEVEL_ERROR) << "Laser info";
		LOG(LEVEL_ERROR) << "Trying to access invalid laser data.";
		
		return -1;
	}
	
	return this->laserReadings.at(value);
}

double Robot::getIfValidLaserReading(unsigned int value)
{
	if (this->laserReadings.size() < value)
	{
		LOG(LEVEL_ERROR) << "Laser info";
		LOG(LEVEL_ERROR) << "Trying to access invalid laser data.";
		
		return -1;
	}
	
	if(laserReadings.at(value) > 3.7)
		return -1;
	
	return this->laserReadings.at(value);
}

void Robot::printLaserReadings()
{
	if (this->laserReadings.size() == 0)
	{
		LOG(LEVEL_ERROR) << "Laser info";
		LOG(LEVEL_ERROR) << "Trying to access laser data while readings "
                         << "array size is zero.";
		
		return;
	}
	
	LOG(LEVEL_WARN) << "Laser info";
	LOG(LEVEL_INFO) << "Laser readings array size = " << this->laserReadings.size();
	
	unsigned int counter;
	for (counter = 0; counter < this->laserReadings.size(); counter++)
	{	
		LOG(LEVEL_INFO) << "Laser[" << counter << "] = " << this->laserReadings.at(counter);
	}
}

void Robot::printValidLaserReadings()
{
	if (this->validLaserReadings.size() == 0)
	{
		LOG(LEVEL_ERROR) << "Valid Laser info";
		LOG(LEVEL_ERROR) << "No valid laser info.";
		
		return;
	}
	
	LOG(LEVEL_WARN) << "Laser info";
	LOG(LEVEL_INFO) << "Valid laser readings array size = " << this->validLaserReadings.size();
	
	unsigned int counter;
	for (counter = 0; counter < this->validLaserReadings.size(); counter++)
	{	
		LOG(LEVEL_INFO) << "Valid Laser[" << counter << "] = " << this->validLaserReadings.at(counter);
	}
}

void Robot::printOneLaserReading(unsigned int value)
{
    double reading = this->getOneLaserReading(value);
    
    if (reading == -1)
    {
        return;
    }
    
	LOG(LEVEL_WARN) << "Laser info";
	LOG(LEVEL_INFO) << "Laser[" << value << "] = " << reading;
}
