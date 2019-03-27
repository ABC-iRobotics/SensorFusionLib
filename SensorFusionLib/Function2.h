#pragma once

#include "Eigen/Dense"

struct Function2 {
	// y = A0*x0 + B0*w0 + f_nl(x0,w0)
	// But f_nl depends on x0_i, w0_j if xDep[i]==1, wDep[i]==1 
	Eigen::MatrixXd A0, B0; // outputsize x inputSize
	Eigen::VectorXi x0Dep, w0Dep; // Constains 1 if the nonlin part depends on it
	typedef std::function < Eigen::VectorXd(const Eigen::VectorXd& x0, const Eigen::VectorXd& w0)> nonlinPart;
	nonlinPart f_nl; // nonlinear part

	Function2(Eigen::MatrixXd A0_, Eigen::MatrixXd B0_, Eigen::VectorXi x0Dep_,
		Eigen::VectorXi w0Dep_, nonlinPart f_);

	unsigned int GetOutputSize() const;

	unsigned int GetX0Size() const;

	unsigned int GetW0Size() const;

	Eigen::VectorXd EvalNl(const Eigen::VectorXd& x0, const Eigen::VectorXd& w0) const;

	Eigen::VectorXd Eval(const Eigen::VectorXd& x0, const Eigen::VectorXd& w0) const;

	static Function2 Empty(unsigned int xSize, unsigned int wSize);
};

