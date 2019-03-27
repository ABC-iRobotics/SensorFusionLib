#include "pch.h"
#include "StatisticValue.h"

#include <exception>

class WrongVarianceSizes : public std::exception {
	virtual const char* what() const throw();
};

const char * WrongVarianceSizes::what() const throw() {
	return "Not consistent value and variance values";
}

StatisticValue::StatisticValue(Eigen::VectorXd vector_, Eigen::MatrixXd variance_) :
	vector(vector_), variance(variance_) {
	if (vector_.size() != variance.rows() || vector_.size() != variance.cols())
		throw WrongVarianceSizes();
}

StatisticValue::StatisticValue(Eigen::VectorXd vector_) :
	vector(vector_), variance(Eigen::MatrixXd::Zero(vector_.size(), vector_.size())) {}

StatisticValue::StatisticValue() : vector(Eigen::VectorXd(0)),
variance(Eigen::MatrixXd(0, 0)) {}

StatisticValue::StatisticValue(unsigned int n) :
	vector(Eigen::VectorXd::Zero(n)), variance(Eigen::MatrixXd::Identity(n, n)) {}

unsigned int StatisticValue::Length() const { return vector.size(); }

void StatisticValue::Insert(unsigned int StartIndex, StatisticValue value) {
	for (unsigned int i = 0; i < value.Length(); i++) {
		vector(StartIndex + i) = value.vector(i);
		for (unsigned int j = 0; j < value.vector.size(); j++)
			variance(StartIndex + i, StartIndex + j) = value.variance(i, j);
		for (unsigned int j = 0; j < StartIndex; j++) {
			variance(j, StartIndex + i) = 0;
			variance(StartIndex + i, j) = 0;
		}
		for (unsigned int j = StartIndex + value.Length(); j < Length(); j++) {
			variance(j, StartIndex + i) = 0;
			variance(StartIndex + i, j) = 0;
		}
	}
}
