#include "pch.h"
#include "Function4.h"
#include "CorrectDep.h"

Function4::Function4(Eigen::MatrixXd A0_, Eigen::MatrixXd B0_, Eigen::MatrixXd Ai_, Eigen::MatrixXd Bi_,
	Eigen::VectorXi x0Dep_, Eigen::VectorXi w0Dep_, Eigen::VectorXi xiDep_, Eigen::VectorXi wiDep_,
	nonlinPart f_) : A0(A0_), Ai(Ai_), B0(B0_), Bi(Bi_),
	x0Dep(CorrectLength(A0_.cols(),x0Dep_)), xiDep(CorrectLength(Ai_.cols(), xiDep_)),
	w0Dep(CorrectLength(B0_.cols(), w0Dep_)), wiDep(CorrectLength(Bi_.cols(), wiDep_)), f_nl(f_) {}

Eigen::Index Function4::GetOutputSize() const { return A0.rows(); }

Eigen::Index Function4::GetX0Size() const { return A0.cols(); }

Eigen::Index Function4::GetW0Size() const { return B0.cols(); }

Eigen::Index Function4::GetXiSize() const { return Ai.cols(); }

Eigen::Index Function4::GetWiSize() const { return Bi.cols(); }

Eigen::VectorXd Function4::EvalNl(const Eigen::VectorXd & x0, const Eigen::VectorXd & w0, const Eigen::VectorXd & xi, const Eigen::VectorXd & wi) const {
	return f_nl(x0, w0, xi, wi);
}

Eigen::VectorXd Function4::Eval(const Eigen::VectorXd & x0, const Eigen::VectorXd & w0, const Eigen::VectorXd & xi, const Eigen::VectorXd & wi) const {
	return A0 * x0 + B0 * w0 +Ai * xi + Bi * wi + f_nl(x0, w0, xi, wi);
}

Function4 Function4::Empty(unsigned int x0Size, unsigned int w0Size, unsigned int xiSize, unsigned int wiSize) {
	return Function4(Eigen::MatrixXd(0, x0Size), Eigen::MatrixXd(0, w0Size),
		Eigen::MatrixXd(0, xiSize), Eigen::MatrixXd(0, wiSize),
		Eigen::VectorXi::Zero(0), Eigen::VectorXi::Zero(0),
		Eigen::VectorXi::Zero(0), Eigen::VectorXi::Zero(0),
		[](const Eigen::VectorXd& baseState, const Eigen::VectorXd& baseDisturbance,
			const Eigen::VectorXd& sensorState, const Eigen::VectorXd& sensorDisturbance) {
		return Eigen::VectorXd(0); });
}
