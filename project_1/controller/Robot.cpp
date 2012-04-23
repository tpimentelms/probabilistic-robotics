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

void Robot::printInfo()
{
	LOG(LEVEL_WARN) << "Robot info";
	LOG(LEVEL_INFO) << "X = " << this->getX();
	LOG(LEVEL_INFO) << "Y = " << this->getY();
	LOG(LEVEL_INFO) << "Th = " << this->getTh();
	LOG(LEVEL_INFO) << "Vel = " << this->getVel();
	LOG(LEVEL_INFO) << "RotVel = " << this->getRotVel();
}

void Robot::getTrueState()
{
	this->setX(p2dProxy.GetXPos());
	this->setY(p2dProxy.GetYPos());
	this->setTh(p2dProxy.GetYaw());
	this->setVel(p2dProxy.GetXSpeed());
	this->setRotVel(p2dProxy.GetYawSpeed());
}

void Robot::setPose(double x, double y, double th)
{
	this->setX(x);
	this->setY(y);
	this->setTh(th);
}

void Robot::setLaserArray(vector<double> measurementArray)
{
	LOG(LEVEL_WARN) << "Laser info";
	LOG(LEVEL_INFO) << "LaserCount = " << laserProxy.GetCount();
	LOG(LEVEL_INFO) << "Laser[0]   = " << laserProxy.GetRange(0);
	this->laserArray = measurementArray;
}





