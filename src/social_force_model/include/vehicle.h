#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <deque>
#include <fstream>

using namespace std;
using namespace Eigen;

class Vehicle{
private:
	static int vehicleIdx;
	int car_id_;
	float length_, width_, max_speed_;
	std::vector<double> pass_possiblity_;
	bool initial_ = false;
	bool ukf_ = false;
	std::fstream data_;
	Color color_;
	Point position_, target_position_;
	std::deque<Point> path_;
	Vector3d velocity_;
	
public:
	Vehicle();
	~Vehicle();
	
	void recount();
	
	void setSize(float l, float w);
	void setPossibility(int i, double y);
	void setPossibility(double y);
	void setSpeed(float speed);
	void setVelocity(Vector3d v);
	void setPosition(float x, float y, float z);
	void setColor(float r, float g, float b);
	void addPath(float x, float y);
	void setUKF();
	
	int getId() { return car_id_; }
	float getLength() { return length_; }
	float getWidth() { return width_; }
	double getPossibility(int i) { return pass_possiblity_[i]; }
	double getPossibility();
	bool isInitial() { return initial_; }
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
	
	double p = DBL_MAX;
	for (int i=0; i<100; ++i)
		pass_possiblity_.push_back(p);

	string file_name = "/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/car_data/car" + std::to_string(car_id_) +".txt";
	data_.open(file_name);
	if(!data_)
		std::cout << "open data file: " << file_name << " failed!" << std::endl;

}

Vehicle::~Vehicle() {
	path_.clear();
	vehicleIdx--;
}

void Vehicle::recount()
{
	vehicleIdx = -1;
}

void Vehicle::setSize(float l, float w)
{
	length_ = l;
	width_ = w;
}

void Vehicle::setPossibility(double y)
{
	for (int i=0; i<100; ++i)
		pass_possiblity_[i] = y;
}

void Vehicle::setPossibility(int i, double y)
{
	pass_possiblity_[i] = y;
}

void Vehicle::setSpeed(float speed)
{
	max_speed_ = speed;
	velocity_[0] = max_speed_;
}

void Vehicle::setVelocity(Vector3d v)
{
	velocity_ = v;
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

void Vehicle::setUKF()
{
	ukf_ = true;
}

double Vehicle::getPossibility()
{
	double p = 1.0;
	for (int i=0; i<pass_possiblity_.size(); ++i)
		if (p > pass_possiblity_[i])
			p = pass_possiblity_[i];
	return p;
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
	acceleration[0] = 8.0F;
	acceleration[1] = 0.0F;
	acceleration[2] = 0.0F;
	
	static double t = 0;

	double desiredSpeed = getPossibility() * max_speed_;
	
	if (velocity_.norm() < desiredSpeed)
		velocity_ = velocity_ + acceleration * stepTime;
	else if (velocity_.norm() > desiredSpeed)
		velocity_ = velocity_ - acceleration * stepTime;
		
	if (velocity_[0] <= 0.001)
		velocity_[0] = 0;
	
	position_.x += velocity_[0] * stepTime;	
	position_.y += velocity_[1] * stepTime;
	t += stepTime;
	if (ukf_ == false)
		data_ << t << " " << position_.x << " " << position_.y
			  << " " << velocity_[0] << " " << velocity_[1] << endl; 
}






















#endif
