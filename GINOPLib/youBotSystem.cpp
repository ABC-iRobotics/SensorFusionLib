#include "youBotSystem.h"

youBotSystem::youBotSystem(double Tdyn, double L, double W, double R) :
	dTdyn(Tdyn), dGeomL(L), dGeomR(R), dGeomW(W) {}

unsigned int youBotSystem::getNumOfDisturbances() const { return 4; }

unsigned int youBotSystem::getNumOfOutputs() const { return 1; }

unsigned int youBotSystem::getNumOfNoises() const { return 0; }

unsigned int youBotSystem::getNumOfStates() const { return 7; }

Eigen::MatrixXd youBotSystem::getA(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Identity(7, 7);
	for (unsigned int i = 0; i < 3; i++)
		out(i, i) -= Ts / dTdyn;
	out(5, 2) = Ts;
	out(6, 6) = 0;
	return out;
}

Eigen::MatrixXd youBotSystem::getB(double Ts) const {
	double a = dGeomR / 4.;
	double b = dGeomR / 2. / (dGeomW + dGeomL);
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(7, 4);
	for (unsigned int i = 0; i < 4; i++) {
		out(0, i) = a / dTdyn * Ts;
		out(1, i) = a / dTdyn * Ts;
		out(2, i) = b / dTdyn * Ts;
		out(6, i) = b;
	}
	out(1, 0) *= -1; out(2, 0) *= -1; out(6, 0) *= -1;
	out(1, 3) *= -1; out(2, 2) *= -1; out(6, 1) *= -1;
	return out;
}

Eigen::MatrixXd youBotSystem::getPInvB(double Ts) const {
	double s = dTdyn / Ts;
	double e = s / dGeomR;
	double g = (dGeomW + dGeomL) / 2. / dGeomR;
	double f = g * s;
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(4, 7);
	for (unsigned int i = 0; i < 4; i++) {
		out(i, 0) = e;
		out(i, 1) = e;
		out(i, 2) = f;
		out(i, 6) = g;
	}
	out(0, 1) *= -1; out(0, 2) *= -1; out(0, 6) *= -1;
	out(3, 1) *= -1; out(2, 2) *= -1; out(1, 6) *= -1;
	return out;
}

Eigen::MatrixXd youBotSystem::getC(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(1, 7);
	out(0, 6) = 1;
	return out;
}

Eigen::MatrixXd youBotSystem::getD(double Ts) const {
	return Eigen::MatrixXd(1, 0);
}

Eigen::VectorXi youBotSystem::getUpdateNonlinearXDependencies() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(7);
	out(0) = 1; out(1) = 1; out(5) = 1;
	return out;
}

Eigen::VectorXd youBotSystem::UpdateNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const {
	Eigen::VectorXd out = Eigen::VectorXd::Zero(7);
	out(3) = Ts * (state(0)*cos(state(5)) - state(1)*sin(state(5)));
	out(4) = Ts * (state(0)*sin(state(5)) + state(1)*cos(state(5)));
	return out;
}

 std::vector<std::string> youBotSystem::getStateNames() const {
	return { "vx","vy","omega","x","y","phi","n" };
}

 std::vector<std::string> youBotSystem::getNoiseNames() const {
	 return {};
 }

 std::vector<std::string> youBotSystem::getDisturbanceNames() const {
	 return { "om_FL","om_FR","om_BL","om_BR" };
 }

 std::vector<std::string> youBotSystem::getOutputNames() const {
	 return { "n" };
 }

 std::string youBotSystem::getName() const {
	 return "youBot";
 }
