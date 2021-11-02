#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "tools.h"
class Traffic_light
{
private:
	static int lightIdx;
	int light_id_;
	Point position_;
	float radius_;
	int now_color_;				//0 red, 1 yellow, 2 green;
	float change_time_, time_left_, time_;	//time_green = time_yellow + time_red
	Color color_;
	
public:
	Traffic_light();
	Traffic_light(float x, float y, int now, float time);
	
	void setRed();
	void setYellow();
	void setGreen();
	
	int getId() { return light_id_; }
	Point getPosition() { return position_; }
	float getRadius() { return radius_; }
	Color getColor() { return color_; }
	float getTimeleft() { return time_left_; }
	void change(float time);
	
};

int Traffic_light::lightIdx = -1;

Traffic_light::Traffic_light()
{
	position_ = setPoint(0.0F, 0.0F, 0.0F);
	time_ = 0.0F;
	color_ = fb_Color(255.0, 0.0, 0.0);
	now_color_ = 0;
	time_ = 0.0F;
}

Traffic_light::Traffic_light(float x, float y, int now, float time)
{
	position_ = setPoint(x, y, 0.0);
	now_color_ = now;
	if (now_color_ == 0)
		color_ = fb_Color(255.0, 0.0, 0.0);
	else if (now_color_ == 1)
		color_ = fb_Color(0.0, 255.0, 255.0);
	else if (now_color_ == 2)
		color_ = fb_Color(0.0, 255.0, 0.0);
	change_time_ = time;
	time_ = 0.0F;
}

void Traffic_light::setRed()
{
	color_ = fb_Color(255.0, 0.0, 0.0);
	now_color_ = 0;
}

void Traffic_light::setYellow()
{
	color_ = fb_Color(0.0, 255.0, 255.0);
	time_ = 0;
	now_color_ = 1;
}

void Traffic_light::setGreen()
{
	color_ = fb_Color(0.0, 255.0, 0.0);
	time_ = 0;
	now_color_ = 2;
}

void Traffic_light::change(float time)
{
	time_ += time;
	if (now_color_ == 0 && time_ >= change_time_)
		setGreen();
	else if (now_color_ == 1 && time_ >= 2.0F)
		setRed();
	else if (now_color_ == 2 && time_ >= change_time_)
		setYellow();
	
	time_left_ = change_time_ - time;
}

#endif





















