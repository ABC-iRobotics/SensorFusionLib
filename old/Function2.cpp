#include "pch.h"
#include "Function2.h"
#include "CorrectDep.h"

Function2::Function2(Eigen::MatrixXd A0_, Eigen::MatrixXd B0_, Eigen::VectorXi x0Dep_,
	Eigen::VectorXi w0Dep_, nonlinPart f_) :
	A0(A0_), B0(B0_), x0Dep(CorrectLength(A0.cols(),x0Dep_)),
	w0Dep(CorrectLength(B0.cols(),w0Dep_)), f_nl(f_) {}

Eigen::Index Function2::GetOutputSize() const { return A0.rows(); }

Eigen::Index Function2::GetX0Size() const { return A0.cols(); }

Eigen::Index Function2::GetW0Size() const { return B0.cols(); }

Eigen::VectorXd Function2::EvalNl(const Eigen::VectorXd & x0, const Eigen::VectorXd & w0) const {
	return f_nl(x0, w0);
}

Eigen::VectorXd Function2::Eval(const Eigen::VectorXd & x0, const Eigen::VectorXd & w0) const {
	return A0 * x0 + B0 * w0 + f_nl(x0, w0);
}

Function2 Function2::Empty(Eigen::Index xSize, Eigen::Index wSize) {
	return Function2(Eigen::MatrixXd(0, xSize), Eigen::MatrixXd(0, wSize),
		Eigen::VectorXi::Zero(xSize), Eigen::VectorXi::Zero(wSize),
		[](const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) {
		return Eigen::VectorXd(0); });
}
