#ifndef CONTROL_H
#define CONTROL_H

#include <vector>
#include "pedestrian.h"
#include "vehicle.h"
#include "traffic_lights.h"
#include "map.h"

class Control{
private:
	std::vector<Pedestrian *> crowd_;
	std::vector<Vehicle *> cars_;
	std::vector<Traffic_light *> lights_;
	Environment *environment_;
	
public:
	~Control();
	
	void addPed(Pedestrian *ped);
	void addCar(Vehicle *car);
	void addLight(Traffic_light *light);
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
	
	void moveCrowd(float stepTime);
	void moveCars(float stepTime);
	void changeLights(float stepTime);
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
