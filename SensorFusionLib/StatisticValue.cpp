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

StatisticValue::StatisticValue(size_t n) :
	vector(Eigen::VectorXd::Zero(n)), variance(Eigen::MatrixXd::Zero(n, n)) {}

Eigen::Index StatisticValue::Length() const { return vector.size(); }

void StatisticValue::Insert(Eigen::Index StartIndex, StatisticValue value) {
	for (Eigen::Index i = 0; i < value.Length(); i++) {
		vector(StartIndex + i) = value.vector(i);
		for (Eigen::Index j = 0; j < value.vector.size(); j++)
			variance(StartIndex + i, StartIndex + j) = value.variance(i, j);
		for (Eigen::Index j = 0; j < StartIndex; j++) {
			variance(j, StartIndex + i) = 0;
			variance(StartIndex + i, j) = 0;
		}
		for (Eigen::Index j = StartIndex + value.Length(); j < Length(); j++) {
			variance(j, StartIndex + i) = 0;
			variance(StartIndex + i, j) = 0;
		}
	}
}

StatisticValue StatisticValue::GetPart(Eigen::Index StartIndex, Eigen::Index Length) const {
	return StatisticValue(vector.segment(StartIndex, Length), variance.block(StartIndex, StartIndex, Length, Length));
}

void StatisticValue::Add(StatisticValue value) {
	Eigen::Index n0 = Length();
	Eigen::Index dn = value.Length();
	vector.conservativeResize(n0 + dn);
	variance.conservativeResize(n0 + dn, n0 + dn);
	Insert(n0, value);
}

std::ostream &operator<<(std::ostream &os, StatisticValue const &m) {
	return os << "Value:\n" << m.vector << "\nVariance:\n" << m.variance;
}
