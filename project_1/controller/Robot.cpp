#include "Robot.hpp"

Robot::Robot()
{
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

double Robot::getVel()
{
	return this->v;
}

double Robot::getRotVel()
{
	return this->w;
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

void Robot::updateState()
{
    // read current state from player client
    playerRobot.Read();
    
	this->setX(p2dProxy.GetXPos());
	this->setY(p2dProxy.GetYPos());
	this->setTh(p2dProxy.GetYaw());
	this->setVel(p2dProxy.GetXSpeed());
	this->setRotVel(p2dProxy.GetYawSpeed());
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

void Robot::updateLaserReadings()
{
	unsigned int counter;
	double laserMeasurement;
	
	this->laserReadings.clear();
	
	for (counter = 0; counter < laserProxy.GetCount(); counter++)
	{
		laserMeasurement = laserProxy.GetRange(counter);
		this->laserReadings.push_back(laserMeasurement);
	}
}

vector<double> Robot::getLaserReadings()
{
	return this->laserReadings;
}

double Robot::getOneLaserReading(unsigned int value)
{
	if (this->laserReadings.size() < value)
	{
		LOG(LEVEL_ERROR) << "Laser info";
		LOG(LEVEL_ERROR) << "Trying to access invalid laser data.";
		
		return -1;
	}
	
	return this->laserReadings.at(250);
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
