#ifndef CONTROL_H
#define CONTROL_H

#include <vector>
#include "pedestrian.h"
#include "vehicle.h"
#include "traffic_lights.h"
#include "evolution.h"
#include "map.h"

class Control{
private:
	float total_time_ = 0.0f;
	
	std::vector<Pedestrian *> crowd_;
	std::vector<Vehicle *> cars_;
	std::vector<Traffic_light *> lights_;
	Environment *environment_;
	Evolution *evolution_;
	
public:
	~Control();
	
	void addPed(Pedestrian *ped);
	void addCar(Vehicle *car);
	void addLight(Traffic_light *light);
	void addEvolution(Evolution *evolution);
	void getEnvironment(Environment *environment);
	
	std::vector<Pedestrian *> getCrowd() { return crowd_; }
	int getCrowdNum() { return crowd_.size(); }
	std::vector<Vehicle *> getCars() { return cars_; }
	int getCarsNum() const { return cars_.size(); }
	std::vector<Traffic_light *> getLights() { return lights_; }
	int getLightsNum() { return lights_.size(); }
	Environment *getEnvironment() { return environment_; }
	
	void removePed();
	void removeCrowd();
	void removeCars();
	void removeLights();
	
	void setInitial();
	void computePoss(float stepTime);
	
	void moveCrowd(float stepTime);
	void moveCars(float stepTime);
	void changeLights(float stepTime);
	void act(float stepTime);
};

Control::~Control()
{
	removeCrowd();
	removeCars();
	removeLights();
}

void Control::addPed(Pedestrian *ped)
{
	crowd_.push_back(ped);
}

void Control::addCar(Vehicle *car)
{
	cars_.push_back(car);
}

void Control::addLight(Traffic_light *light)
{
	lights_.push_back(light);
}

void Control::addEvolution(Evolution *evolution)
{
	evolution_ = evolution;
}

void Control::getEnvironment(Environment *environment)
{
	environment_ = environment;
}

void Control::removeCrowd() 
{
	for (int idx = 0; idx < crowd_.size(); idx++)
		delete crowd_[idx];
	crowd_.clear();
}

void Control::removePed() 
{
	int lastIdx;

	if (!crowd_.empty()) {
		lastIdx = crowd_.size() - 1;

		delete crowd_[lastIdx];
		crowd_.pop_back();
	}
}

void Control::removeCars() 
{
	for (int idx = 0; idx < cars_.size(); idx++)
		delete cars_[idx];
	cars_.clear();
}

void Control::removeLights()
{
	for (int idx = 0; idx < lights_.size(); idx++)
		delete lights_[idx];
	lights_.clear();
}

void Control::setInitial()
{
	for (Vehicle *car_i: cars_)
	{
		for (Pedestrian *ped_i: crowd_)
		{
			if (ped_i->isInitial())
				continue;
			car_i->setPossibility(ped_i->getId(), 0.5);
			cout << "car initial possibility setted:" << ped_i->getId() << '\t'<< 0.5 << endl;
			double p = ( 4.0 - 2 * (-2.0 - ped_i->getPosition().y) + ped_i->getVelocity()[1] * ped_i->getMeetTime()) / 8.0;
			ped_i->setPossibility(p);
			cout << "ped " << ped_i->getId() << " initial possibility setted:" << p << endl;
		}
	}
}

void Control::computePoss(float stepTime)
{
	//cout<<"computing evolution path!" << endl;
	for (Vehicle *car_i: cars_)
	{
		for (Pedestrian *ped_i: crowd_)
		{
			evolution_->setInitial(total_time_, ped_i->getPossibility(), car_i->getPossibility(ped_i->getId()));
			evolution_->setStep(stepTime * 5);
			evolution_->RK4();
			ped_i->setPossibility(evolution_->getX());
			car_i->setPossibility(ped_i->getId(), evolution_->getY());
			//cout<<"x1= "<< evolution_->getX() << "\ty1= " << evolution_->getY()<<endl; 
		}
	}
	//cout<<"computing done!"<<endl;
}

void Control::act(float stepTime)
{
	evolution_->meet(crowd_, cars_);
	if(evolution_->isConflict())
	{
		setInitial();
		computePoss(stepTime);
	}
	
	if(evolution_->isEnd())
	{
		for (Vehicle *car_i: cars_)
			car_i->setPossibility(1);
		for (Pedestrian *ped_i: crowd_)
			ped_i->setPossibility(1);
	}
	
	
	moveCrowd(stepTime);
	moveCars(stepTime);
	changeLights(stepTime);
	
	total_time_ += stepTime;
}

void Control::moveCrowd(float stepTime)
{
	for (int idx = 0; idx < crowd_.size(); ++idx)
		crowd_[idx]->move(crowd_, environment_->getWalls(), environment_->getZebras(), lights_, stepTime);
		
}

void Control::moveCars(float stepTime) {
	for (int idx = 0; idx < cars_.size(); idx++)
		cars_[idx]->move(stepTime);
}

void Control::changeLights(float stepTime)
{
	for (int idx = 0; idx < lights_.size(); idx++)
		lights_[idx]->change(stepTime);
}







#endif
