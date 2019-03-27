#pragma once
#include "Eigen/Dense"

struct Function4 {
	// y = A0*x0 + Ai*xi + B0*w0 + Bi*wi + f_nl(x0,xi,w0,wi)
	// But f_nl depends on x0_i, w0_j if xDep[i]==1, wDep[i]==1

	Eigen::MatrixXd A0, B0, Ai, Bi; // outputsize x inputSize

	Eigen::VectorXi x0Dep, w0Dep, xiDep, wiDep; // Constains 1 if the nonlin part depends on it

	typedef std::function < Eigen::VectorXd(const Eigen::VectorXd& x0, const Eigen::VectorXd& w0,
		const Eigen::VectorXd& xi, const Eigen::VectorXd& wi)> nonlinPart;

	nonlinPart f_nl; // nonlinear part

	Function4(Eigen::MatrixXd A0_, Eigen::MatrixXd B0_, Eigen::MatrixXd Ai_, Eigen::MatrixXd Bi_,
		Eigen::VectorXi x0Dep_, Eigen::VectorXi w0Dep_, Eigen::VectorXi xiDep_,
		Eigen::VectorXi wiDep_, nonlinPart f_);

	unsigned int GetOutputSize() const;

	unsigned int GetX0Size() const;

	unsigned int GetW0Size() const;

	unsigned int GetXiSize() const;

	unsigned int GetWiSize() const;

	Eigen::VectorXd EvalNl(const Eigen::VectorXd& x0, const Eigen::VectorXd& w0,
		const Eigen::VectorXd& xi, const Eigen::VectorXd& wi) const;

	Eigen::VectorXd Eval(const Eigen::VectorXd& x0, const Eigen::VectorXd& w0,
		const Eigen::VectorXd& xi, const Eigen::VectorXd& wi) const;

	static Function4 Empty(unsigned int x0Size, unsigned int w0Size, unsigned int xiSize, unsigned int wiSize);
};
