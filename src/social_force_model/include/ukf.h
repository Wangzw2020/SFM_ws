#ifndef UKF_H
#define UKF_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include "control.h"

using namespace std;
using namespace Eigen;

Pedestrian *ped;

struct StateInfo{
	int id;
	double p_x;
	double p_y;
	double v_x;
	double v_y;
};

class UKF{
private:
	static int targetIdx;
	bool initialized_;
	int stateDimension_;
	int sigmaPointsNum_;
	double alpha_, beta_, lambda_;
	int kappa_;
	
	std::vector<StateInfo> all_targets_;

	VectorXd state_;
	MatrixXd stateCovariance_;
	MatrixXd predictedSigmaPoints_;
	MatrixXd predictedstateSigmaPoint_;

	VectorXd stateWeights_;
	VectorXd stateCovarianceWeights_;
	
	int measurementDimension_;
	VectorXd measurementState_;
	MatrixXd measurementNoise_;
	MatrixXd measurementSigmaPoints_;
	MatrixXd measurementMatrix_;
	
	bool initialize(VectorXd initialState, MatrixXd initialStateCovariance);
	void updateNum();
	
	MatrixXd computeSigmaPoints(VectorXd initialState, MatrixXd initialStateCovariance);
	MatrixXd sigmaPointPrediction(MatrixXd &sigmaPoints, const double &deltaTime);
	
	VectorXd calculateState(VectorXd &state, MatrixXd &predictedSigmaPoints);
	MatrixXd calculateCovariance(MatrixXd &stateCovariance, MatrixXd &predictedSigmaPoints, VectorXd &predictedState, MatrixXd &processNoise);
	MatrixXd calculateProcessNoise();
	
	VectorXd setStateWeights();
	VectorXd setCovarianceWeights();
	
	Control *control_;
	
public:
	UKF();
	~UKF();

	Control *getControl() { return control_; }

	bool isInitialized() { return initialized_; }
	bool initialize(VectorXd firstState, Control *c);
	VectorXd getState() { return state_; }
	MatrixXd getStateCovariance() { return stateCovariance_; }
	MatrixXd getPredictedSigmaPoints() { return predictedSigmaPoints_; }
	MatrixXd predict(const double deltaTime);
	void setMeasurement(VectorXd measurement, MatrixXd measurementNoise, int dimention);
	void update();
	bool isVectorValid(const VectorXd testVector);
	bool doesVectorContainValues(const VectorXd testVector);
	bool isMatrixValid(const MatrixXd testMatrix);
	bool doesMatrixContainValues(const MatrixXd testMatrix);
	
};

int UKF::targetIdx = -1;

UKF::UKF()
{
	alpha_ = 0.1;		//0 < alpha <= 1
	beta_ = 2;			//for Gaussian distributions b = 2 is optimal
	stateDimension_ = 4 * all_targets_.size();
	kappa_ = 0.0;		//k is a secondary scaling parameter
	sigmaPointsNum_ = 2 * stateDimension_ + 1;
	measurementDimension_ = 2;
	//lambda是缩放因子，控制sigma点与均值的分离距离
	lambda_ = alpha_ * alpha_ * (stateDimension_ + kappa_) - stateDimension_;
	
	initialized_ = false;
	
	state_.resize(stateDimension_);
	state_.fill(0.0);
	stateCovariance_.resize(stateDimension_, stateDimension_);
	stateCovariance_.fill(0.0);
	predictedSigmaPoints_.resize(stateDimension_, sigmaPointsNum_);
	predictedSigmaPoints_.fill(0.0);
	predictedstateSigmaPoint_.resize(stateDimension_, sigmaPointsNum_);
	predictedstateSigmaPoint_.fill(0.0);
}

UKF::~UKF()
{

}

bool UKF::initialize(VectorXd firstState, Control *c)
{
	if (initialized_ == true)
		return initialized_;
	
	if (!isVectorValid(firstState))
		return false;
	
	VectorXd tmp_firstState;
	tmp_firstState = firstState;
	
	ped = new Pedestrian;
	
	control_ = c;
	
	int num = firstState.size();
	num = num/4;
	for (int i=0; i<num; ++i)
	{
		++targetIdx;
		StateInfo state_i;
		state_i.id = targetIdx;
		state_i.p_x = firstState(0+4*i);
		state_i.p_y = firstState(1+4*i);
		state_i.v_x = firstState(2+4*i);
		state_i.v_y = firstState(3+4*i);
		all_targets_.push_back(state_i);
		ped->setGroupId(1);
		ped->setPosition(tmp_firstState(0+4*i), tmp_firstState(1+4*i));
		ped->addPath(tmp_firstState(0+4*i), tmp_firstState(1+4*i) + 50);
		ped->setUKF();
		control_->addPed(ped);
	}
	
	updateNum();
	
	initialized_ = true;
	
	cout << "alpha = " << alpha_ << '\n'
		 << "beta  = " << beta_ << '\n'
		 << "kappa = " << kappa_ << '\n'
		 << "lambda= " << lambda_ << endl;
		 
	cout << "ukf initialized!" << endl;
	cout << "firstState: \n" << state_ << endl;
	return initialized_;
}

void UKF::updateNum()
{
	int num = all_targets_.size();
	stateDimension_ = 4 * num;
	sigmaPointsNum_ = 2 * stateDimension_ + 1;

	lambda_ = alpha_ * alpha_ * (stateDimension_ + kappa_) - stateDimension_;
	
	state_.resize(stateDimension_);
	measurementMatrix_.resize(measurementDimension_, stateDimension_);
	measurementMatrix_.fill(0);
	for (int i=0; i<num; ++i)
	{
		state_(0+4*i) = all_targets_[i].p_x;
		state_(1+4*i) = all_targets_[i].p_y;
		state_(2+4*i) = all_targets_[i].v_x;
		state_(3+4*i) = all_targets_[i].v_y;
		measurementMatrix_(0, 0+4*i) = 1;
		measurementMatrix_(1, 1+4*i) = 1;
	}
	
	cout << "measurement matrix: \n" << measurementMatrix_ << endl;
	
	stateWeights_ = setStateWeights();
	cout << "state weights:  \n" << stateWeights_.transpose() << endl;
	stateCovarianceWeights_ = setCovarianceWeights();
	cout << "state covariance weights:  \n" << stateCovarianceWeights_.transpose() << endl;
	
	stateCovariance_.resize(stateDimension_, stateDimension_);
/*	stateCovariance_ = 0.1 * Eigen::MatrixXd::Identity(stateDimension_, stateDimension_);*/
	stateCovariance_.fill(0.0);
	for(int i=0; i<num; ++i)
	{
		stateCovariance_(0+4*i, 0+4*i) = 0.01;
		stateCovariance_(1+4*i, 1+4*i) = 0.01;
		stateCovariance_(2+4*i, 2+4*i) = 0.01;
		stateCovariance_(3+4*i, 3+4*i) = 0.01;
	}
	cout << "state covariance:  \n" << stateCovariance_ << endl;
	
	predictedSigmaPoints_.resize(stateDimension_, sigmaPointsNum_);
	predictedSigmaPoints_.fill(0.0);
	predictedstateSigmaPoint_.resize(stateDimension_, sigmaPointsNum_);
	predictedstateSigmaPoint_.fill(0.0);
}

void UKF::setMeasurement(VectorXd measurement, MatrixXd measurementNoise, int dimention)
{
	measurementState_ = measurement;
	cout << "Measurement State : \n" << measurementState_.transpose() << endl;
	measurementNoise_ = measurementNoise;
	measurementDimension_ = dimention;
	predictedstateSigmaPoint_ = computeSigmaPoints(state_, stateCovariance_);
	measurementSigmaPoints_ = predictedstateSigmaPoint_;
	measurementSigmaPoints_ = measurementMatrix_ * predictedstateSigmaPoint_;
	
	cout << "predictedstate Sigma Points : \n" << predictedstateSigmaPoint_ << endl;
	cout << "Measurement Sigma Points : \n" << measurementSigmaPoints_ << endl;
}

VectorXd UKF::setStateWeights()
{
	VectorXd weights(sigmaPointsNum_);
	weights.fill(0.0);
	
	double firstWeight = lambda_ / (lambda_ + stateDimension_);
	weights(0) = firstWeight;
	for (int i=1; i<sigmaPointsNum_; ++i)
	{
		double weight = 1 / (2 * (stateDimension_ + lambda_));
		weights(i) = weight;
	}
	return weights;
}

VectorXd UKF::setCovarianceWeights()
{
	VectorXd weights(sigmaPointsNum_);
	weights.fill(0.0);
	
	double firstWeight = lambda_ / (lambda_ + stateDimension_) + 1 - pow(alpha_,2) + beta_;
	weights(0) = firstWeight;
	for (int i=1; i<sigmaPointsNum_; ++i)
	{
		double weight = 1 / (2 * (stateDimension_ + lambda_));
		weights(i) = weight;
	}
	return weights;
}

MatrixXd UKF::predict(const double deltaTime)
{
	cout << "predict!" << endl;
	if (!isInitialized())
		cout << "UKF is not initialized!" << endl;
		
	if (deltaTime < 0)
		cout << "deltaTime < 0!" << endl;

	MatrixXd sigmaPoints = computeSigmaPoints(state_, stateCovariance_);
	cout << "sigmaPoints: \n" << sigmaPoints <<endl;
	
	predictedSigmaPoints_ = sigmaPointPrediction(sigmaPoints, deltaTime);
	cout << "predictedSigmaPoints: \n" << predictedSigmaPoints_ <<endl;

	VectorXd predictedState = calculateState(state_, predictedSigmaPoints_);
	cout << "predictedState: \n" << predictedState.transpose() <<endl;

	MatrixXd processNoise = calculateProcessNoise();
	cout << "processNoise: \n" << processNoise <<endl;

	MatrixXd predictedStateCovariance = calculateCovariance(stateCovariance_, predictedSigmaPoints_, predictedState, processNoise);
	cout << "predictedStateCovariance: \n" << predictedStateCovariance <<endl;

	if (!(isVectorValid(predictedState) || isMatrixValid(predictedStateCovariance)))
	{
		cout << "prediction failed!" << endl;
	}
	state_ = predictedState;
	stateCovariance_ = predictedStateCovariance;

	return predictedSigmaPoints_;
}

MatrixXd UKF::computeSigmaPoints(VectorXd initialState, MatrixXd initialStateCovariance)
{
	MatrixXd sigmaPoints(stateDimension_, sigmaPointsNum_);

	MatrixXd squareRootMatrix(stateDimension_, stateDimension_);
	squareRootMatrix.fill(0.0);
	squareRootMatrix = initialStateCovariance.llt().matrixL();
	
	sigmaPoints.col(0) = initialState;
	
	for (int i = 0; i<stateDimension_; ++i)
	{
		sigmaPoints.col(i+1) = initialState + sqrt(lambda_ + stateDimension_) * squareRootMatrix.col(i);
		sigmaPoints.col(i+1+stateDimension_) = initialState - sqrt(lambda_ + stateDimension_) * squareRootMatrix.col(i);
	}
/*	cout << "stateDimension: " << stateDimension_ << endl;*/
/*	cout << "now state: \n" << initialState << endl;*/
/*	cout << "sigma points: \n" << sigmaPoints << endl;*/
	
	return sigmaPoints;
}

MatrixXd UKF::sigmaPointPrediction(MatrixXd &sigmaPoints, const double &deltaTime)
{
	MatrixXd predictedSigmaPoints(stateDimension_, sigmaPointsNum_);
	predictedSigmaPoints.fill(0.0);
	vector<Pedestrian *> crowds = control_->getCrowd();
	
	for(int i = 0; i<sigmaPointsNum_; ++i)
	{
		double p_x = sigmaPoints(0,i);
		double p_y = sigmaPoints(1,i);
		double v_x = sigmaPoints(2,i);
		double v_y = sigmaPoints(3,i);
		for (Pedestrian *ped : crowds)
		{
			ped->setPosition(p_x, p_y);
			ped->setVelocity(v_x, v_y);
		}
		control_->act(deltaTime);
		
		for (Pedestrian *ped : crowds)
		{
			predictedSigmaPoints(0, i) = ped->getPosition().x;
			predictedSigmaPoints(1, i) = ped->getPosition().y;
			predictedSigmaPoints(2, i) = ped->getVelocity()[0];
			predictedSigmaPoints(3, i) = ped->getVelocity()[1];
		}
	}
	
	return predictedSigmaPoints;
}

MatrixXd UKF::calculateProcessNoise()
{
	MatrixXd A(stateDimension_,stateDimension_);
	A.fill(0.0);
	for (int i=0; i<stateDimension_; ++i)
	{
		A(i,i) = 0.01;
	}

	return A;
}

void UKF::update()
{
	if (!isInitialized())
		cout << "UKF is not initialized!" << endl;
	
	if (!isVectorValid(measurementState_))
		cout << "The measurement vector is not valid!" << endl;
		
	if (!doesVectorContainValues(measurementState_))
		cout << "The measurement vector is empty!" << endl;
		
	if (!isMatrixValid(measurementNoise_))
		cout << "The measurementNoise matrix is not valid!" << endl;
	
	if (!isMatrixValid(measurementSigmaPoints_))
		cout << "The measurementSigmaPoints matrix is not valid!" << endl;
		
	if (!isMatrixValid(predictedSigmaPoints_))
		cout << "The predictedSigmaPoints matrix is not valid!" << endl;
		
	if (measurementDimension_ <= 0)
		cout << "The measurementDimension is not permitted!" << endl;
	else if (measurementDimension_ > stateDimension_)
		cout << "New target Found!" << endl;
	
	VectorXd predictedMeasurements(measurementDimension_);
	predictedMeasurements.fill(0.0);
	MatrixXd measurementCovariance(measurementDimension_, measurementDimension_);
	measurementCovariance.fill(0.0);
	MatrixXd crossCorrelation(stateDimension_, measurementDimension_);
    crossCorrelation.fill(0.0);

	predictedMeasurements = calculateState(predictedMeasurements, measurementSigmaPoints_);
	cout << "predictedMeasurements: \n" << predictedMeasurements <<endl;

	measurementCovariance = calculateCovariance(measurementCovariance, measurementSigmaPoints_, predictedMeasurements, measurementNoise_);
	cout << "measurementCovariance: \n" << measurementCovariance <<endl;
	
	VectorXd measurementDifference(sigmaPointsNum_);
	measurementDifference.fill(0.0);
	VectorXd stateDifference(sigmaPointsNum_);
	stateDifference.fill(0.0);
	
	for(int i = 0; i<sigmaPointsNum_; ++i)
	{
		measurementDifference = measurementSigmaPoints_.col(i) - predictedMeasurements;
/*		stateDifference = predictedSigmaPoints_.col(i) - state_;*/
		stateDifference = predictedstateSigmaPoint_.col(i) - state_;
		crossCorrelation += stateCovarianceWeights_(i) * stateDifference * measurementDifference.transpose();
	}
	cout << "crossCorrelation: \n" << crossCorrelation <<endl;
	
	if (measurementCovariance.determinant() == 0)
		cout << "Taking the inverse of a 0 Matrix is prohibited!" << endl;

	MatrixXd KalmanGain = crossCorrelation * measurementCovariance.inverse();
	cout << "KalmanGain: \n" << KalmanGain <<endl;
	
	measurementDifference = measurementState_ -  predictedMeasurements;
	
	VectorXd temp_state = state_;
	temp_state += KalmanGain * measurementDifference;
	cout << "corrected state: \n" << temp_state <<endl;

	MatrixXd temp_stateCovaiance = stateCovariance_ - (KalmanGain * measurementCovariance * KalmanGain.transpose());
	//cout << "corrected stateCovaiance: \n" << temp_stateCovaiance <<endl;
	
	if (!(isMatrixValid(temp_stateCovaiance) || isVectorValid(temp_state)))
		cout << "update failed! corrected data has illegal values!" << endl;
	
	state_ = temp_state;
	stateCovariance_ = temp_stateCovaiance;
	
	for (int i=0; i< all_targets_.size(); ++i)
	{
		all_targets_[i].p_x = state_(0+4*i);
		all_targets_[i].p_y = state_(1+4*i);
		all_targets_[i].v_x = state_(2+4*i);
		all_targets_[i].v_y = state_(3+4*i);
	}
}

MatrixXd UKF::calculateCovariance(MatrixXd &stateCovariance, MatrixXd &predictedSigmaPoints, VectorXd &predictedState, MatrixXd &processNoise)
{
	MatrixXd predictedStateCovariance(stateCovariance.rows(), stateCovariance.cols());
	predictedStateCovariance.fill(0.0);
	
	for(int i=0; i<sigmaPointsNum_; ++i)
	{
		VectorXd stateDifference = predictedSigmaPoints.col(i) - predictedState;
		predictedStateCovariance += stateCovarianceWeights_(i) * stateDifference * stateDifference.transpose();
	}
	predictedStateCovariance += processNoise;
	return predictedStateCovariance;
}

VectorXd UKF::calculateState(VectorXd &state, MatrixXd &predictedSigmaPoints)
{
	VectorXd predictedState = state;
	predictedState.fill(0.0);
	for(int i=0; i<sigmaPointsNum_; ++i)
	{
		predictedState += stateWeights_(i) * predictedSigmaPoints.col(i);
	}
	return predictedState;
}

bool UKF::isVectorValid(const VectorXd testVector)
{
	for(int i = 0; i < testVector.size(); ++i)
		if (std::isnan(testVector(i)) || std::isinf(testVector(i)))
			return false;
	return true;
}

bool UKF::doesVectorContainValues(const VectorXd testVector)
{
	for(int i = 0; i < testVector.size(); ++i)
		if (testVector(i) !=0 )
			return true;
	return false;
}

bool UKF::isMatrixValid(const MatrixXd testMatrix)
{
	for(int i = 0; i < testMatrix.rows(); ++i)
		for(int j = 0; j < testMatrix.cols(); j++)
			if (std::isnan(testMatrix(i,j)) || std::isinf(testMatrix(i,j)))
				return false;
	return true;
}

bool UKF::doesMatrixContainValues(const MatrixXd testMatrix)
{
	for(int i = 0; i < testMatrix.rows(); ++i)
		for(int j = 0; j < testMatrix.cols(); j++)
			if (testMatrix(i,j) != 0)
				return true;
	return false;
}


#endif
