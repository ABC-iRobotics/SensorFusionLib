#pragma once

#include "Eigen/Dense"
#include <vector>
#include "Function2.h"
#include "Function4.h"
#include "StatisticValue.h"


// To temporaly merge system matrices and wrap the nonlinear parts to one general function
class FunctionMerge {
	Function2 f0; // params: x0,w0
	std::vector<Function4> fi; // params: x0,w0,xi,wi

public:
	FunctionMerge(const Function2& f0_);;

	void AddFunction(const Function4& f);

	unsigned int GetOutputSize() const;

	unsigned int GetXInputSize() const;

	unsigned int GetWInputSize() const;

	Eigen::VectorXd Eval(const Eigen::VectorXd& x, const Eigen::VectorXd& w) const;

	Eigen::VectorXd EvalNl(const Eigen::VectorXd& x, const Eigen::VectorXd& w) const;

	Eigen::MatrixXd GetA() const;

	Eigen::MatrixXd GetB() const;

	Eigen::VectorXi GetXDep() const;

	Eigen::VectorXi GetWDep() const;

	Eigen::MatrixXd GetXSelectorNl() const;

	Eigen::MatrixXd GetXSelectorL() const;

	Eigen::MatrixXd GetWSelectorNl() const;

	Eigen::MatrixXd GetWSelectorL() const;

	StatisticValue Eval(const StatisticValue& x, const StatisticValue& w, Eigen::MatrixXd& Syx, Eigen::MatrixXd& Syw) const;
};
