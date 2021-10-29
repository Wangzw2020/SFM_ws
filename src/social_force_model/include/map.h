#include <vector>
#include "tools.h"

class Wall{
private:
	Line wall_;	
public:
	Wall(float x1, float y1, float x2, float y2, float h);
	
	float getH() { return wall_.start.z; }
	Point getStartPoint() { return wall_.start; }
	Point getEndPoint() { return wall_.end; }
	Point getNearestPoint(Point p);
};

class Seperator{
private:
	Line sep_;	
public:
	Seperator(float x1, float y1, float x2, float y2);
	
	Point getStartPoint() { return sep_.start; }
	Point getEndPoint() { return sep_.end; }
};

class Zebra{
private:
	Point position_;
	float length_, width_, interval_;
	Color color_;
public:
	Zebra(float x, float y);
	Point getPosition() { return position_; }
	float getLength() { return length_; }
	float getWidth() { return width_; }
	float getInterval() { return interval_; }
	Color getColor() { return color_; }
};

class Environment{
private:
	std::vector<Wall *> walls_;
	std::vector<Seperator *> seps_;
	std::vector<Zebra *> zebras_;
public:
	~Environment();
	
	void addWall(Wall *wall);
	void addSep(Seperator *sep);
	void addZebra(Zebra *zebra);
	
	const std::vector<Wall *> getWalls() { return walls_; }
	int getNumWalls() { return walls_.size(); }
	const std::vector<Seperator *> getSeps() { return seps_; }
	int getNumSeps() { return seps_.size(); }
	const std::vector<Zebra *> getZebras() { return zebras_; }
	int getNumZebras() { return zebras_.size(); }
	
	void removeWalls();
	void removeSeps();
	void removeZebras();
};

Wall::Wall(float x1, float y1, float x2, float y2, float h)
{
	wall_.start = setPoint(x1, y1, h);
	wall_.end = setPoint(x2, y2, h);
}

Point Wall::getNearestPoint(Point p)
{
	Eigen::Vector3d relativeEnd, relativePos, relativeEndScal, relativePosScal;
	float dotProduct;
	Point nearestPoint;
	
	relativeEnd = setVector(wall_.end, wall_.start);
	relativePos = setVector(p, wall_.start);
	relativeEndScal = relativeEnd;
	relativeEndScal.normalize();
	relativePosScal = relativePos * (1.0f / relativeEnd.norm());
	
	dotProduct = relativeEndScal.dot(relativePosScal);
	
	if (dotProduct < 0.0)
		nearestPoint = wall_.start;
	else if (dotProduct > 1.0)
		nearestPoint = wall_.end;
	else
	{
		nearestPoint.x = (relativeEnd[0] * dotProduct) + wall_.start.x;
		nearestPoint.y = (relativeEnd[1] * dotProduct) + wall_.start.y;
		nearestPoint.z = (relativeEnd[2] * dotProduct) + wall_.start.z;
	}
	return nearestPoint;
	
}

Seperator::Seperator(float x1, float y1, float x2, float y2)
{
	sep_.start = setPoint(x1, y1, 0.0);
	sep_.end = setPoint(x2, y2, 0.0);
}

Zebra::Zebra(float x, float y)
{
	position_ = setPoint(x, y, 0.0);
	color_.r = 150.0 / 255.0;
	color_.g = 150.0 / 255.0;
	color_.b = 150.0 / 255.0;
	length_ = 4.0F;
	width_ = 0.4F;
	interval_ = 0.4F;
}

Environment::~Environment()
{
	removeWalls();
	removeSeps();
	removeZebras();
}

void Environment::addWall(Wall *wall)
{
	walls_.push_back(wall);
}

void Environment::addSep(Seperator *sep)
{
	seps_.push_back(sep);
}

void Environment::addZebra(Zebra *zebra)
{
	zebras_.push_back(zebra);
}

void Environment::removeWalls()
{
	for (int i=0; i<walls_.size(); ++i)
		delete walls_[i];
	walls_.clear();
}

void Environment::removeSeps()
{
	for (int i=0; i<seps_.size(); ++i)
		delete seps_[i];
	seps_.clear();
}

void Environment::removeZebras()
{
	for (int i=0; i<zebras_.size(); ++i)
		delete zebras_[i];
	zebras_.clear();
}
