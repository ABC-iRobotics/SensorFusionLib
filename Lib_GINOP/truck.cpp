#include "truck.h"

Truck::Truck(double Tdyn, double W, double R, unsigned int ID) : BaseSystem(ID),
	dTdyn(Tdyn), dGeomR(R), dGeomW(W) {}

unsigned int Truck::getNumOfDisturbances() const { return 2; }

unsigned int Truck::getNumOfOutputs() const { return 0; }

unsigned int Truck::getNumOfNoises() const { return 0; }

unsigned int Truck::getNumOfStates() const { return 6; }

Eigen::MatrixXd Truck::getA(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Identity(6, 6);
	for (unsigned int i = 0; i < 3; i++)
		out(i, i) -= Ts / dTdyn;
	out(5, 2) = Ts;
	return out;
}

Eigen::MatrixXd Truck::getB(double Ts) const {
	double c = dGeomR * Ts / dTdyn;
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(6, 2);
	out(0, 0) = c / 2.; out(0, 1) = c / 2.;
	out(2, 0) = -c / dGeomW; out(2, 1) = c / dGeomW;
	return out;
}

Eigen::MatrixXd Truck::getPInvB(double Ts) const {
	double c = dTdyn / (dGeomR * Ts * 2);
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(2, 6);
	out(0, 0) = c * 2.; out(1, 0) = c * 2.;
	out(0, 2) = -c * dGeomW; out(1, 2) = c * dGeomW;
	return out;
}

Eigen::MatrixXd Truck::getC(double Ts) const {
	return Eigen::MatrixXd::Zero(0, 6);
}

Eigen::MatrixXd Truck::getD(double Ts) const {
	return Eigen::MatrixXd(0, 0);
}

Eigen::VectorXi Truck::getStateUpdateNonlinXDep() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(6);
	out(0) = 1; out(1) = 1; out(5) = 1;
	return out;
}

Eigen::VectorXd Truck::EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const {
	Eigen::VectorXd out = Eigen::VectorXd::Zero(6);
	out(3) = Ts * (state(0)*cos(state(5)) - state(1)*sin(state(5)));
	out(4) = Ts * (state(0)*sin(state(5)) + state(1)*cos(state(5)));
	return out;
}

 std::vector<std::string> Truck::getStateNames() const {
	return { "vx","vy","omega","x","y","phi" };
}

 std::vector<std::string> Truck::getNoiseNames() const {
	 return {};
 }

 std::vector<std::string> Truck::getDisturbanceNames() const {
	 return { "om_L","om_R" };
 }

 std::vector<std::string> Truck::getOutputNames() const {
	 return {};
 }

 std::string Truck::getName() const {
	 return "Truck";
 }
