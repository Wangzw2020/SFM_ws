#ifndef DATA_H
#define DATA_H

#include <vector>
#include "tools.h"
struct Info{
	double time;
	double x;
	double y;
};

class Data{
private:
	int type_;		//0 for ped  //1 for car
	int id_;
	int num_;
	std::vector<Info> data_;
public:
	Data();
	~Data();
	
	void setType(int i);
	int getType();
	void addData(Info info);
	Info getData(int i) { return data_[i]; }
	int getNum() { return num_; }
	std::vector<Info> getData() { return data_; }
};

Data::Data()
{
	type_ = 0;
	id_ = 0;
	num_ = 0;
}

Data::~Data()
{
	data_.clear();
}

void Data::setType(int i)
{
	type_ = i;
}

int Data::getType()
{
	return type_;
}

void Data::addData(Info info)
{
	num_++;
	data_.push_back(info);
}

#endif 
