#ifndef UKF_H
#define UKF_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include "control.h"

using namespace std;
using namespace Eigen;

class UKF{
private:
	bool initialized_;
	int stateDimension_;
	int sigmaPointsNum_;
	double alpha_, beta_, lambda_;
	int kappa_;
	
	VectorXd state_;
	MatrixXd stateCovariance_;
	MatrixXd predictedSigmaPoints_;
	VectorXd stateWeights_;
	VectorXd stateCovarianceWeights_;
	
	bool initialize(VectorXd initialState, MatrixXd initialStateCovariance);
	MatrixXd computeSigmaPoints(VectorXd initialState, MatrixXd initialStateCovariance);
	MatrixXd sigmaPointPrediction(MatrixXd &sigmaPoints, const double &deltaTime);
	
	VectorXd calculateState(VectorXd &state, MatrixXd &predictedSigmaPoints);
	MatrixXd predictCovariance(MatrixXd &stateCovariance, MatrixXd &predictedSigmaPoints, VectorXd &predictedState, MatrixXd &processNoise);
	MatrixXd calculateProcessNoise(const double &deltaTime, const double &yaw);
	
	VectorXd setStateWeights();
	VectorXd setCovarianceWeights();
	
	Control *control_;
	
public:
	UKF();
	~UKF();
	
	void setControl(Control *c);
	Control *getControl() { return control_; }
	
	bool isInitialized() { return initialized_; }
	bool initialize(VectorXd firstState);
	VectorXd getState() { return state_; }
	MatrixXd getStateCovariance() { return stateCovariance_; }
	MatrixXd getPredictedSigmaPoints() { return predictedSigmaPoints_; }
	MatrixXd predict(const double deltaTime);
	void update(VectorXd &measurements, MatrixXd &measurementNoise,
                MatrixXd &measurementSigmaPoints, MatrixXd predictedSigmaPoints,
                const int measurementDimension);
	bool isVectorValid(const VectorXd testVector);
	bool doesVectorContainValues(const VectorXd testVector);
	bool isMatrixValid(const MatrixXd testMatrix);
	bool doesMatrixContainValues(const MatrixXd testMatrix);
	
};

UKF::UKF()
{
	alpha_ = 0.1;		//0 < alpha <= 1
	beta_ = 2;			//for Gaussian distributions b = 2 is optimal
	stateDimension_ = 4;
	kappa_ = 0.0;		//k is a secondary scaling parameter
	sigmaPointsNum_ = 2 * stateDimension_ + 1;
	//lambda是缩放因子，控制sigma点与均值的分离距离
	lambda_ = alpha_ * alpha_ * (stateDimension_ + kappa_) - stateDimension_;
	
	initialized_ = false;
	
	state_.resize(stateDimension_);
	state_.fill(0.0);
	stateCovariance_.resize(stateDimension_, stateDimension_);
	stateCovariance_.fill(0.0);
	predictedSigmaPoints_.resize(stateDimension_, sigmaPointsNum_);
	predictedSigmaPoints_.fill(0.0);
	
}

UKF::~UKF()
{

}

void UKF::setControl(Control *c)
{
	control_ = c;
}

bool UKF::initialize(VectorXd firstState)
{
	if (initialized_ == true)
		return initialized_;
	
	if (!isVectorValid(firstState))
		return false;
		
	stateWeights_ = setStateWeights();
	stateCovarianceWeights_ = setCovarianceWeights();
	
	state_ = firstState;
	stateCovariance_ = 100 * Eigen::MatrixXd::Identity(stateDimension_, stateDimension_);
	
	initialized_ = true;
	return initialized_;
	
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
	//cout << "weights: \n" << weights.transpose() <<endl;
	return weights;
}

VectorXd UKF::setCovarianceWeights()
{
	VectorXd weights(sigmaPointsNum_);
	weights.fill(0.0);
	
	double firstWeight = lambda_ / (lambda_ + stateDimension_) + 1 - pow(alpha_,2) + beta_;
	weights(0) = firstWeight;
	for (int i=0; i<sigmaPointsNum_; ++i)
	{
		double weight = 1 / (2 * (stateDimension_ + lambda_));
		weights(i) = weight;
	}
	return weights;
}

MatrixXd UKF::predict(const double deltaTime)
{
	if (!isInitialized())
		cout << "UKF is not initialized!" << endl;
		
	if (deltaTime < 0)
		cout << "deltaTime < 0 !" << endl;
	
	MatrixXd sigmaPoints = computeSigmaPoints(state_, stateCovariance_);
	//cout << "sigmaPoints: \n" << sigmaPoints <<endl;
	
	predictedSigmaPoints_ = sigmaPointPrediction(sigmaPoints, deltaTime);
	cout << "predictedSigmaPoints: \n" << predictedSigmaPoints_ <<endl;
	
	VectorXd predictedState = calculateState(state_, predictedSigmaPoints_);
	cout << "predictedState: \n" << predictedState.transpose() <<endl;
	
	MatrixXd processNoise = calculateProcessNoise(deltaTime, predictedState(2));
	//cout << "processNoise: \n" << processNoise <<endl;
	
	MatrixXd predictedStateCovariance = predictCovariance(stateCovariance_, predictedSigmaPoints_, predictedState, processNoise);
	//cout << "predictedStateCovariance: \n" << predictedStateCovariance <<endl;

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
	MatrixXd squareRootMatrix = initialStateCovariance.llt().matrixL();
	
	sigmaPoints.col(0) = initialState;
	
	for (int i = 0; i<stateDimension_; ++i)
	{
		sigmaPoints.col(i+1) = initialState + sqrt(lambda_ + stateDimension_) * squareRootMatrix.col(i);
		sigmaPoints.col(i+1+stateDimension_) = initialState - sqrt(lambda_ + stateDimension_) * squareRootMatrix.col(i);
	}
	
	return sigmaPoints;
}

MatrixXd UKF::sigmaPointPrediction(MatrixXd &sigmaPoints, const double &deltaTime)
{
	MatrixXd predictedSigmaPoints(stateDimension_, sigmaPointsNum_);
	predictedSigmaPoints.fill(0.0);
	vector<Pedestrian *> crowds = control_->getCrowd();
	
	for(int i = 0; i<sigmaPointsNum_; ++i)
	{
		const double p_x = sigmaPoints(0,i);
		const double p_y = sigmaPoints(1,i);
		const double v_x = sigmaPoints(2,i);
		const double v_y = sigmaPoints(3,i);
		control_->act(deltaTime);
		
		for (Pedestrian *ped : crowds)
			if (ped->getId() == control_->getTargetId())
			{
				predictedSigmaPoints(0, i) = ped->getPosition().x;
				predictedSigmaPoints(1, i) = ped->getPosition().y;
				predictedSigmaPoints(2, i) = ped->getVelocity()[0];
				predictedSigmaPoints(3, i) = ped->getVelocity()[1];
			}
	}
	
	return predictedSigmaPoints;
}

MatrixXd UKF::calculateProcessNoise(const double &deltaTime, const double &yaw)
{
	MatrixXd A(4,4);
	A.fill(0.01);
	//???
	return A;
}

void UKF::update(VectorXd &measurements, MatrixXd &measurementNoise,
				MatrixXd &measurementSigmaPoints, MatrixXd predictedSigmaPoints,
				const int measurementDimension)
{
	if (!isInitialized())
		cout << "UKF is not initialized!" << endl;
	
	if (!isVectorValid(measurements))
		cout << "The measurement vector is not valid!" << endl;
		
	if (!doesVectorContainValues(measurements))
		cout << "The measurement vector is empty!" << endl;
		
	if (!isMatrixValid(measurementNoise))
		cout << "The measurementNoise matrix is not valid!" << endl;
	
	if (!isMatrixValid(measurementSigmaPoints))
		cout << "The measurementSigmaPoints matrix is not valid!" << endl;
		
	if (!isMatrixValid(predictedSigmaPoints))
		cout << "The predictedSigmaPoints matrix is not valid!" << endl;
		
	if (measurementDimension <= 0 || measurementDimension > stateDimension_)
		cout << "The measurementDimension is not permitted!";
	
	VectorXd predictedMeasurements(measurementDimension);
	predictedMeasurements.fill(0.0);
	MatrixXd measurementCovariance(measurementDimension, measurementDimension);
	measurementCovariance.fill(0.0);
	MatrixXd crossCorrelation(stateDimension_, measurementDimension);
    crossCorrelation.fill(0.0);

	predictedMeasurements = calculateState(predictedMeasurements, measurementSigmaPoints);
	//cout << "predictedMeasurements: \n" << predictedMeasurements <<endl;

	measurementCovariance = predictCovariance (measurementCovariance, measurementSigmaPoints, predictedMeasurements, measurementNoise);
	//cout << "measurementCovariance: \n" << measurementCovariance <<endl;
	
	VectorXd measurementDifference;
	measurementDifference.fill(0.0);
	VectorXd stateDifference;
	stateDifference.fill(0.0);
	
	for(int i = 0; i<sigmaPointsNum_; ++i)
	{
		measurementDifference = measurementSigmaPoints.col(i) - predictedMeasurements;
		stateDifference = predictedSigmaPoints.col(i) - state_;
		crossCorrelation += stateCovarianceWeights_(i) * stateDifference * measurementDifference.transpose();
	}
	//cout << "crossCorrelation: \n" << crossCorrelation <<endl;
	
	if (measurementCovariance.determinant() == 0)
        cout << "Taking the inverse of a 0 Matrix is prohibited!" << endl;

	MatrixXd KalmanGain = crossCorrelation * measurementCovariance.inverse();
	//cout << "KalmanGain: \n" << KalmanGain <<endl;
	
	cout << "measurement state: \n" << measurements.transpose() <<endl;
	measurementDifference = measurements - predictedMeasurements;
	
	VectorXd temp_state = state_;
	temp_state += KalmanGain * measurementDifference;
	cout << "corrected state: \n" << temp_state <<endl;

	MatrixXd temp_stateCovaiance = stateCovariance_ - (KalmanGain * measurementCovariance * KalmanGain.transpose());
	//cout << "corrected stateCovaiance: \n" << temp_stateCovaiance <<endl;
	
	if (!(isMatrixValid(temp_stateCovaiance) || isVectorValid(temp_state)))
		cout << "update failed! corrected data has illegal values!" << endl;
	
	state_ = temp_state;
	stateCovariance_ = temp_stateCovaiance;
}

MatrixXd UKF::predictCovariance(MatrixXd &stateCovariance, MatrixXd &predictedSigmaPoints, VectorXd &predictedState, MatrixXd &processNoise)
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
