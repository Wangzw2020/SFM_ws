#ifndef EVOLUTION_H
#define EVOLUTION_H

class Evolution{
private:
	std::vector<int> ped_id_;
	int car_id_;
	double J_, N_, A_, B_;
	double x_initial_, y_initial_, t_initial_;
	double x_now_, y_now_, t_now_;
	double step_;
	bool conflict_ = false, end_ = false;
	
public:
	Evolution();
	~Evolution();
	void meet(std::vector<Pedestrian *> crowd, std::vector<Vehicle *> cars);
	
	void setParam(double J, double N, double A, double B);
	void setInitial(double t, double x, double y);
	void setStep(double h);
	bool isConflict() { return conflict_; }
	bool isEnd() { return end_; }
	
	std::vector<int> getPedId() { return ped_id_; }
	int getCarId() { return car_id_; }
	double getX() { return x_now_; }
	double getY() { return y_now_; }
	
	double f(double x, double y);
	double g(double x, double y);
	void RK4();
	
	
};

Evolution::Evolution()
{
	
}

Evolution::~Evolution()
{

}

void Evolution::meet(std::vector<Pedestrian *> crowd, std::vector<Vehicle *> cars)
{
	double x1, y1, k1, x2, y2, k2;
	Point p;
	Eigen::Vector3d dis_car, dis_ped, dis_between;
	double t_car, t_ped;
	
	for (Vehicle *car_i: cars)
	{
		x1 = car_i->getPosition().x;
		y1 = car_i->getPosition().y;
		k1 = car_i->getVelocity()[1] / car_i->getVelocity()[0];
		for (Pedestrian *ped_i: crowd)
		{
			dis_between = setVector(ped_i->getPosition(), car_i->getPosition());
			if(	dis_between.norm() >= 20.0 )
			{
				conflict_ = false;
				continue;
			}
			x2= ped_i->getPosition().x;
			y2 = ped_i->getPosition().y;
			k2 = ped_i->getVelocity()[1] / ped_i->getVelocity()[0];
			
			p.x = (y2 - y1 + k1 * x1 - k2 * x2) / (k1 - k2);
			p.y = (k1 * k2 * (x2 - x1) + k2 * y1 - k1 * y2) / (k2 - k1);
			if (k1 * (p.x-x1) + y1 + 1.75 >= p.y && k1 * (p.x-x1) + y1 - 1.75 <= p.y)
			{
				dis_car = setVector(car_i->getPosition(), p);
				dis_ped = setVector(ped_i->getPosition(), p);
				t_car = dis_car.norm() / car_i-> getVelocity().norm();
				if ( car_i->getPosition().x > p.x + 1.0 || ped_i->getPosition().y > p.y + 1.75)
				{
					conflict_ = false;
					end_ = true;
				}
					
				else
				{
/*					t_ped = dis_ped.norm() / ped_i->getVelocity().norm();*/
					t_ped = dis_ped.norm() / ped_i->getDesiredSpeed();
					ped_i->setMeetTime(abs(t_ped));
					//cout<<t_car - t_ped<<endl;
					if (t_car - t_ped <= 1.6 && t_car - t_ped >= -1.6)
					{
						car_id_ = car_i->getId();
						ped_id_.push_back(ped_i->getId());
						conflict_ = true;
					}
				}
			}
		}
	}
}

void Evolution::setParam(double J, double N, double A, double B)
{
	J_ = J;
	N_ = N;
	A_ = A;
	B_ = B;
}

void Evolution::setInitial(double t, double x, double y)
{
	t_initial_ = t;
	x_initial_ = x;
	y_initial_ = y;
}

void Evolution::setStep(double h)
{
	step_ = h;
}

double Evolution::f(double x, double y)
{
	double dx;
	dx = x * (1-x) * (J_ - (J_+A_) * y);
	return (dx);
}

double Evolution::g(double x, double y)
{
	double dy;
	dy = y * (1-y) * (N_ - (N_+B_) * x);
	return (dy);
}

void Evolution::RK4()
{
	//cout<<"t= "<< step_<<endl;
	//cout<<"x1= "<< x_initial_ << "\ty1= " << y_initial_<<endl; 
	double f1, f2, f3, f4, g1, g2, g3, g4;
	f1 = f(x_initial_, y_initial_);
	g1 = g(x_initial_, y_initial_);
	f2 = f(x_initial_ + step_ * f1 / 2, y_initial_ + step_ * g1 / 2);
	g2 = g(x_initial_ + step_ * f1 / 2, y_initial_ + step_ * g1 / 2);
	f3 = f(x_initial_ + step_ * f2 / 2, y_initial_ + step_ * g2 / 2);
	g3 = g(x_initial_ + step_ * f2 / 2, y_initial_ + step_ * g2 / 2);
	f4 = f(x_initial_ + step_ * f3, y_initial_ + step_ * g3);
	g4 = g(x_initial_ + step_ * f3, y_initial_ + step_ * g3);
	t_now_ = t_initial_ + step_;
	x_now_ = x_initial_ + step_ * (f1 + 2*f2 + 2*f3 + f4) / 6;
	y_now_ = y_initial_ + step_ * (g1 + 2*g2 + 2*g3 + g4) / 6;
	//cout<<"x2= "<< x_now_ << "\ty2= " << y_now_<<endl; 
}

















































#endif
