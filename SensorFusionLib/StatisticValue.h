#pragma once

#include "Eigen/Dense"

#include <iostream>

struct StatisticValue {
	Eigen::VectorXd vector;

	Eigen::MatrixXd variance;

	bool isIndependent = false;

	StatisticValue(Eigen::VectorXd vector_, Eigen::MatrixXd variance_);

	StatisticValue(Eigen::VectorXd vector_);

	StatisticValue(size_t n);

	StatisticValue();

	Eigen::Index Length() const;

	void Insert(Eigen::Index StartIndex, StatisticValue value);

	StatisticValue GetPart(Eigen::Index StartIndex, Eigen::Index Length) const;

	void Add(StatisticValue value);

};

std::ostream &operator<<(std::ostream &os, StatisticValue const &m);
