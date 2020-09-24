#include "Vechicle2D.h"

using namespace SF;

Eigen::MatrixXd SF::Vechicle2D::getA(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(5, 5);
	out(2, 4) = Ts;
	return out;
}

Eigen::MatrixXd SF::Vechicle2D::getB(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(5, 2);
	out(3, 0) = Ts;
	out(4, 1) = Ts;
#ifdef use_Ts2_coeffs
	out(2, 1) = Ts * Ts / 2.;
#endif
	return out;
}

Eigen::VectorXd SF::Vechicle2D::EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd & state, const Eigen::VectorXd & disturbance) const {
	static Eigen::VectorXd out = Eigen::VectorXd::Zero(5);
	double ds = Ts * state(3);
#ifdef use_Ts2_coeffs
	ds += Ts * Ts / 2.*disturbance(0);
#endif
	out(0) = ds * cos(state(2));
	out(1) = ds * sin(state(2));
	return out;
}

// The nonlinear parts and the dependencies are by default zeros

Eigen::VectorXi SF::Vechicle2D::getStateUpdateNonlinXDep() const {
	static Eigen::VectorXi out = Eigen::VectorXi::Zero(5);
	out(2) = 1;
	out(3) = 1;
	return out;
}

#ifdef use_Ts2_coeffs
Eigen::VectorXi SF::VelocityBased_2DBase::getStateUpdateNonlinWDep() const {
	static Eigen::VectorXi out = Eigen::VectorXi::Zero(2);
	out(0) = 1;
	return out;
}
#endif

Eigen::MatrixXd SF::Vechicle2D::getC(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 5);
	for (int i = 0; i < 3; i++)
		out(i, i) = 1.;
	return out;
}

Eigen::MatrixXd SF::Vechicle2D::getD(double Ts) const {
	static auto out = Eigen::MatrixXd::Zero(3, 0);
	return out;
}

unsigned int SF::Vechicle2D::getNumOfStates() const {
	return 5;
}

unsigned int SF::Vechicle2D::getNumOfDisturbances() const {
	return 2;
}

unsigned int SF::Vechicle2D::getNumOfOutputs() const {
	return 3;
}

unsigned int SF::Vechicle2D::getNumOfNoises() const {
	return 0;
}

Eigen::VectorXi ZZOZZ() {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(5);
	out(2) = 1;
	return out;
}

const Eigen::VectorXi & SF::Vechicle2D::getIfStateIsRad() const {
	static Eigen::VectorXi out = ZZOZZ();
	return out;
}
