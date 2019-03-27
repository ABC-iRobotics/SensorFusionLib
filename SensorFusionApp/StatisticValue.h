#pragma once

#include "Eigen/Dense"

struct StatisticValue {
	Eigen::VectorXd vector;

	Eigen::MatrixXd variance;

	bool isIndependent = false;

	StatisticValue(Eigen::VectorXd vector_, Eigen::MatrixXd variance_);

	StatisticValue(Eigen::VectorXd vector_);

	StatisticValue(unsigned int n);

	StatisticValue();

	unsigned int Length() const;

	void Insert(unsigned int StartIndex, StatisticValue value);
};

