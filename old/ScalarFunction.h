#pragma once

#include "Eigen/Core"

// Only for inner usage - a descriptor will be used with more arguments to describe the sensor characteristics
/*
class ScalarFunction
{
public:
	ScalarFunction();
	~ScalarFunction();

	// y = f(x), it recieves all of the states
	virtual double getValue(Eigen::VectorXd value) const = 0;

	// If the expected values of the state is x, with variance matrix S
	// [y_expected,S_yy,S_yx] = f*(x_expected,S_xx)
	virtual void getStatistics(Eigen::VectorXd value, Eigen::MatrixXd variance,
		double & result, Eigen::VectorXd & res_variance, Eigen::VectorXd & res_covariance) const  = 0;

	// For linearized based EKF
	virtual void getLinearized(Eigen::VectorXd value, double & result, Eigen::VectorXd & gradient) const = 0;
};

class Sin1Function : public ScalarFunction
{
private:
	int iIndex;
	double dOmega;
	double dOffset;
public:
	Sin1Function(int i, double omega, double offset): ScalarFunction() {
		iIndex = i;
		dOmega = omega;
		dOffset = offset;
	};

	double getValue(Eigen::VectorXd value) const override { return sin(value[iIndex] * dOmega); };

	// If the expected values of the state is x, with variance matrix S
	// [y_expected,S_yy,S_yx] = f*(x_expected,S_xx)
	void getStatistics(Eigen::VectorXd value, Eigen::MatrixXd variance,
		double & result, Eigen::VectorXd & res_variance, Eigen::VectorXd & res_covariance) const override {
		result = sin(dOmega*value[iIndex] + dOffset)*exp(-dOmega * dOmega / variance[iIndex][iIndex] / variance[iIndex][iIndex] / 2.);
		
	};
};

class Cos1Function : public Sin1Function
{
public:
	Cos1Function(int i, double omega, double offset) : Sin1Function(i, omega, offset + EIGEN_PI / 2.) {};
};
*/
