#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <deque>

using namespace std;
using namespace Eigen;

class Vehicle{
private:
	static int vehicleIdx;
	int car_id_;
	float length_, width_, max_speed_;
	Color color_;
	Point position_, target_position_;
	std::deque<Point> path_;
	Vector3d velocity_;
	
public:
	Vehicle();
	~Vehicle();
	
	void setSize(float l, float w);
	void setSpeed(float speed);
	void setPosition(float x, float y, float z);
	void setColor(float r, float g, float b);
	void addPath(float x, float y);
	
	int getId() { return car_id_; }
	float getLength() { return length_; }
	float getWidth() { return width_; }
	float getMaxSpeed() { return max_speed_; }
	Point getPosition() { return position_; }
	Color getColor() { return color_; }
	Point getPath();
	Vector3d getVelocity() { return velocity_; }
	
	void move(float stepTime);
};

int Vehicle::vehicleIdx = -1;

Vehicle::Vehicle() {
	car_id_ = ++vehicleIdx;

	length_ = 3.2F;
	width_ = 2.2F;

	setColor(0.0, 0.0, 0.0);
	setPosition(0.0, 0.0, 0.0);
	velocity_ = Eigen::Vector3d::Zero(); 
}

Vehicle::~Vehicle() {
	path_.clear();
	vehicleIdx--;
}

void Vehicle::setSize(float l, float w)
{
	length_ = l;
	width_ = w;
}

void Vehicle::setSpeed(float speed)
{
	max_speed_ = speed;
}

void Vehicle::setPosition(float x, float y, float z)
{
	position_ = setPoint(x, y, z);
}

void Vehicle::setColor(float r, float g, float b)
{
	color_ = fb_Color(r, g, b);
}

void Vehicle::addPath(float x, float y)
{
	Point p = setPoint(x, y, -0.01);
	path_.push_back(p);
}

Point Vehicle::getPath()
{
	Vector3d dis_now, dis_next;
	dis_now = setVector(position_, path_[0]);
	
	if (path_.size() > 2)
	{
		dis_next = setVector( position_, path_[1]);
		if (dis_next.norm() < dis_now.norm())
		{
			path_.push_back(path_.front());
			path_.pop_front();
			dis_now = dis_next;
		}
	}
	if (dis_now.norm() < 2.0)
	{
		path_.push_back(path_.front());
		path_.pop_front();
	}
	return path_.front();	
}

void Vehicle::move(float stepTime)
{
	Vector3d acceleration;
	acceleration[0] = 5.0F;
	acceleration[1] = 0.0F;
	acceleration[2] = 0.0F;
	velocity_ += acceleration * stepTime;
	if (velocity_.norm() > max_speed_)
		velocity_[0] = max_speed_;
	position_.x += velocity_[0] * stepTime;	
	position_.y += velocity_[1] * stepTime;	
}






















#endif
