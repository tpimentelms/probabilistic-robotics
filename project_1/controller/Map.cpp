#include "Map.hpp"

Map::Map()
{
	point zero;
	
	zero.x = 0;
	zero.y = 0;
}


Map::~Map()
{
}
/*
 * TODO: Implement all functions that are described at Map.hpp
 */

void Map::createKnownMap()
{
	point wallPointBegin, wallPointEnd;
	
	wallPointBegin.x = -5;
	wallPointBegin.y = 2.6;
	
	wallPointBegin.x = -5;
	wallPointBegin.y = 12;
	
	this->addCorner(wallPointBegin);
	
	this->addWall(wallPointBegin, wallPointEnd);
	
	
	wallPointBegin.x = -5;
	wallPointBegin.y = 2.6;
	
	wallPointEnd.x = -5;
	wallPointEnd.y = 12;
	
	this->addCorner(wallPointBegin);
	
	this->addWall(wallPointBegin, wallPointEnd);
	
	wallPointBegin = wallPointEnd;
	
	wallPointEnd.x = 12.1;
	wallPointEnd.y = 12;
	
	this->addCorner(wallPointBegin);
	
	this->addWall(wallPointBegin, wallPointEnd);
	
	wallPointBegin = wallPointEnd;
	
	wallPointEnd.x = 12.1;
	wallPointEnd.y = -12.5;
	
	this->addCorner(wallPointBegin);
	
	this->addWall(wallPointBegin, wallPointEnd);
	
	wallPointBegin = wallPointEnd;
	
	wallPointEnd.x = -12.3;
	wallPointEnd.y = -12.5;
	
	this->addCorner(wallPointBegin);
	
	this->addWall(wallPointBegin, wallPointEnd);
	
	wallPointBegin = wallPointEnd;
	
	wallPointEnd.x = -12.3;
	wallPointEnd.y = 2.6;
	
	this->addCorner(wallPointBegin);
	
	this->addWall(wallPointBegin, wallPointEnd);
	
	wallPointBegin = wallPointEnd;
	
	wallPointEnd.x = -5;
	wallPointEnd.y = 2.6;
	
	this->addCorner(wallPointBegin);
	
	this->addWall(wallPointBegin, wallPointEnd);
	
	
}

void Map::addWall(point begin, point end)
{
	wall newWall;
	newWall.begin = begin;
	newWall.end = end;
	this->walls.push_back(newWall);
}

void Map::addCorner(point corner)
{
	this->corners.push_back(corner);
}
