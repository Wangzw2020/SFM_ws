#ifndef PEDESTRAIN_H
#define PEDESTRAIN_H

#include <random>
#include <deque>
#include <vector>
#include <fstream>
#include "map.h"
#include "traffic_lights.h"

class Pedestrian{
private:
	static int crowdIdx;
	int ped_id_, group_id_;
	float radius_;
	float react_time_, desiredSpeed_;
	std::fstream data_;
	Color color_;
	Point position_;
	std::deque<Point> path_;
	Eigen::Vector3d velocity_;
	
	Eigen::Vector3d drivingForce();
	float A_ped_, B_ped_, lambda_, K, k;
	Eigen::Vector3d pedInteractForce(std::vector<Pedestrian *> ped);
	float A_wall_, B_wall_;
	Eigen::Vector3d wallInteractForce(std::vector<Wall *> walls);
	float A_g_, B_g_;
	Eigen::Vector3d groupForce(std::vector<Pedestrian *> ped);
	float A_z1_, B_z1_, A_z2_, B_z2_;
	bool near_zebra(std::vector<Zebra *> zebras);
	bool in_zebra;
	int zebra_id_ = -1;
	Eigen::Vector3d zebraForce(std::vector<Zebra *> zebras);
	bool see_light(std::vector<Traffic_light *> lights);
	int light_id_ = -1;
	Eigen::Vector3d lightForce(std::vector<Traffic_light *> lights);
	
public:
	Pedestrian();
	~Pedestrian();
	
	void setRadius(float r);
	void setDesiredSpeed(float speed);
	void setColor(float r, float g, float b);
	void setPosition(float x, float y);
	void addPath(float x, float y);
	void setGroupId(int id);
	void setLightId(int id);
	
	int getId() { return ped_id_; }
	int getGroupId() { return group_id_; }
	int getLightId() { return light_id_; }
	float getRadius() { return radius_; }
	float getDesiredSpeed() { return desiredSpeed_; }
	Color getColor() { return color_; }
	Point getPosition() { return position_; }
	Point getPath();
	Eigen::Vector3d getVelocity() { return velocity_; }
	float getOrientation();
	
	void move(std::vector<Pedestrian *> crowd, std::vector<Wall *> walls, std::vector<Zebra *> zebras, std::vector<Traffic_light *> lights , float stepTime);
};

int Pedestrian::crowdIdx = -1;

Pedestrian::Pedestrian()
{
	ped_id_ = ++crowdIdx;
	std::ifstream param_txt;
	param_txt.open("/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/param.txt");
	if(!param_txt)
		std::cout << "open param file failed!" << std::endl;
	string line;
	while(param_txt.good())
	{
		getline(param_txt,line);
		if (line.length() == 0)
			break;
		std::stringstream ss(line);
		ss 	>> radius_ >> react_time_ >> A_ped_ >> B_ped_ >> lambda_ >> K >> k
			>> A_wall_ >> B_wall_ >> A_g_ >> B_g_ >> A_z1_ >> B_z1_ >> A_z2_ >> B_z2_;
	}	
	param_txt.close();
	
	string file_name = "/home/wzw/workspace/SFM_ws/src/social_force_model/src/files/ped_data/ped" + std::to_string(ped_id_) +".txt";
	data_.open(file_name);
	//cout<<file_name<<endl;
	if(!data_)
		std::cout << "open data file failed!" << std::endl;

	desiredSpeed_ = randomFloat(1.1F, 1.4F);
	
	color_ = fb_Color(0.0, 0.0, 0.0);
	position_ = setPoint(0.0, 0.0, 0.0);
	velocity_ = Eigen::Vector3d::Zero();;
}

Pedestrian::~Pedestrian()
{
	path_.clear();
	crowdIdx--;
	data_.close();
}

void Pedestrian::setRadius(float r)
{
	radius_ = r;
}

void Pedestrian::setDesiredSpeed(float speed)
{
	desiredSpeed_ = speed;
}

void Pedestrian::setColor(float r, float g, float b)
{
	color_ = fb_Color(r, g, b);
}

void Pedestrian::setPosition(float x, float y)
{
	position_ = setPoint(x, y, 0.0);
}

void Pedestrian::addPath(float x, float y)
{
	Point p;
	p.x = x;
	p.y = y;
	path_.push_back(p);
}

void Pedestrian::setGroupId(int id)
{
	group_id_ = id;
}

void Pedestrian::setLightId(int id)
{
	light_id_ = id;
}

Point Pedestrian::getPath()
{
	Eigen::Vector3d dis_now, dis_next;
	
	dis_now = setVector(position_, path_[0]);
	
	if (path_.size() > 2)
	{
		dis_next = setVector(position_, path_[1]);
		if (dis_next.norm() < dis_now.norm())
		{
			path_.push_back(path_.front());
			path_.pop_front();
			dis_now = dis_next;
		}	
	}
	if (dis_now.norm() < 2.0F)
	{
		path_.push_back(path_.front());
		path_.pop_front();
	}
	return path_.front();
}

float Pedestrian::getOrientation()
{
	return (atan2(velocity_[1], velocity_[0]) * (180 / PI));
}

void Pedestrian::move(std::vector<Pedestrian *> crowd, std::vector<Wall *> walls, std::vector<Zebra *> zebras, std::vector<Traffic_light *> lights , float stepTime)
{
	Eigen::Vector3d acceleration;
	
	acceleration = drivingForce() + pedInteractForce(crowd) + wallInteractForce(walls) + groupForce(crowd) +  zebraForce(zebras);
	velocity_ = velocity_ + acceleration * stepTime;
	
	position_.x = position_.x + velocity_[0] * stepTime;
	position_.y = position_.y + velocity_[1] * stepTime;
}

Eigen::Vector3d Pedestrian::drivingForce()
{
	Eigen::Vector3d e_i, f_i;
	e_i = setVector(position_, getPath());
	e_i.normalize();
	
	f_i = ((desiredSpeed_ * e_i) - velocity_) * (1 / react_time_);
	return f_i;
}

Eigen::Vector3d Pedestrian::pedInteractForce(std::vector<Pedestrian *> ped)
{
	Eigen::Vector3d dis_ij, n_ij, e_ij, F1, F2, F3, f_ij;
	float F_ij;
	f_ij = Eigen::Vector3d::Zero();
	
	for (const Pedestrian *ped_j : ped)
		if (ped_j->ped_id_ != ped_id_)
		{
			F1 = Eigen::Vector3d::Zero();
			F2 = Eigen::Vector3d::Zero();
			F3 = Eigen::Vector3d::Zero();
			dis_ij = setVector(ped_j->position_, position_);
			if (dis_ij.norm() > 3.0)
				continue;
			
			n_ij = dis_ij;
			n_ij.normalize();
			if (velocity_.norm() != 0)		
			{
				e_ij = velocity_;
				e_ij.normalize();
				F_ij = lambda_ + (1-lambda_)*(1-n_ij.dot(e_ij)) / 2;
			}
			else
				F_ij = lambda_ + (1-lambda_)/2;
			F1 = A_ped_ * exp((radius_ + ped_j->radius_ - dis_ij.norm()) / B_ped_) * n_ij * F_ij;
			if (dis_ij.norm() < radius_ + ped_j->radius_)
			{
				F2 = K * (radius_ + ped_j->radius_) * n_ij;
				//F3
			}
			f_ij += F1 + F2 + F3;
		}
	return f_ij;
}

Eigen::Vector3d Pedestrian::wallInteractForce(std::vector<Wall *> walls)
{
	Point nearestPoint;
	Eigen::Vector3d v_wi, min_v_wi;
	float dis, min_dis = INFINITY, d_w,f_iw;
	
	for (Wall *wall : walls)
	{
		nearestPoint = wall->getNearestPoint(position_);
		v_wi = setVector(nearestPoint, position_);
		dis = v_wi.norm();
		if (dis < min_dis)
		{
			min_dis = dis;
			min_v_wi = v_wi;
		}
	}
	d_w = min_dis - radius_;
	
	f_iw = A_wall_ * exp(-d_w / B_wall_);
	min_v_wi.normalize();
	
	return f_iw * min_v_wi;
}

Eigen::Vector3d Pedestrian::groupForce(std::vector<Pedestrian *> ped)
{
	Point center;
	int num = 1;
	float c_x = position_.x, c_y = position_.y;
	Eigen::Vector3d dis_ic, f_g, n_ic;
	f_g = Eigen::Vector3d::Zero();
	for (Pedestrian *ped_j : ped)
		if (ped_j->ped_id_ != ped_id_)
			if (ped_j->group_id_ == group_id_)
			{
				c_x += ped_j->getPosition().x;
				c_y += ped_j->getPosition().y;
				++num;
			}
	center = setPoint(c_x/num, c_y/num, 0.0);
	dis_ic = setVector(position_, center);
	n_ic = dis_ic;
	n_ic.normalize();
	if (num > 1)
		f_g = A_g_ * exp((dis_ic.norm() - radius_) * B_g_) * n_ic;
	else
		f_g = Eigen::Vector3d::Zero();
	return f_g;
}

bool Pedestrian::near_zebra(std::vector<Zebra *> zebras)
{
	for (Zebra *zebra_i: zebras)
		if (position_.x >= zebra_i->getLeftDown().x - 0.5 && position_.x <= zebra_i->getRightUp().x + 0.5)
			if (position_.y >= zebra_i->getLeftDown().y -0.5 && position_.y <= zebra_i->getRightUp().y + 0.5)
			{
				zebra_id_ = zebra_i->getId();
				if (position_.x >= zebra_i->getLeftDown().x && position_.x <= zebra_i->getRightUp().x)
					in_zebra = true;
				else
					in_zebra = false;
				return true;
			}
	zebra_id_ = -1;
	return false;
}

Eigen::Vector3d Pedestrian::zebraForce(std::vector<Zebra *> zebras)
{
	Eigen::Vector3d f_z = Eigen::Vector3d::Zero();
	if(!near_zebra(zebras))
		return f_z;
	Eigen::Vector3d e_iz = Eigen::Vector3d::Zero();
	float dis_to_z;
	for (Zebra *zebra_i: zebras)
		if (zebra_i->getId() == zebra_id_)
		{
			e_iz[0] = zebra_i->getPosition().x - position_.x;
			dis_to_z = e_iz.norm();
			e_iz.normalize();
			if (in_zebra)
				if (velocity_.dot(e_iz) <= 0)
					f_z = A_z2_ * exp(B_z2_ * dis_to_z) * e_iz;
				else
					f_z = Eigen::Vector3d::Zero();
			else
				f_z = A_z1_ * exp(B_z1_ * dis_to_z) * e_iz;
		}
	return f_z;
}

bool Pedestrian::see_light(std::vector<Traffic_light *> lights)
{
	return false;	
	
	return true;
}

Eigen::Vector3d Pedestrian::lightForce(std::vector<Traffic_light *> lights)
{
	Eigen::Vector3d f_l, P;
	f_l = Eigen::Vector3d::Zero();
	if (!see_light(lights))
		return f_l;
	
	float time_limit;
	
	return f_l;
}

#endif

