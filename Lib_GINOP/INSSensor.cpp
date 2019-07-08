#include "INSSensor.h"
#include "KUKAyouBot.h"
#include "truck.h"

Eigen::MatrixXd INSSensor::getAs_bs(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 7);
}

Eigen::MatrixXd INSSensor::getAs(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd INSSensor::getBs_bs(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 4);
}

Eigen::MatrixXd INSSensor::getBs(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd INSSensor::getPInvBs(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd INSSensor::getCs_bs(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 7);
	out(2, 5) = 1;
	return out;
}

Eigen::MatrixXd INSSensor::getCs(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd INSSensor::getDs_bs(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 0);
}

Eigen::MatrixXd INSSensor::getDs(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 0);
}

Eigen::VectorXi INSSensor::getStateUpdateNonlinXbsDep() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(7);
	out(0) = 1; out(1) = 1; out(5) = 1;
	return out;
}

INSSensor::INSSensor(BaseSystem::BaseSystemPtr ptr, unsigned int ID) : Sensor(ptr, ID) {}

unsigned int INSSensor::getNumOfStates() const {
	return 3;
}

unsigned int INSSensor::getNumOfDisturbances() const {
	return 3;
}

unsigned int INSSensor::getNumOfOutputs() const {
	return 3;
}

unsigned int INSSensor::getNumOfNoises() const {
	return 0;
}

Eigen::VectorXi INSSensor::getStateUpdateNonlinXsDep() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(3);
	out(2) = 1;
	return out;
}

Eigen::VectorXd INSSensor::EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd & baseSystemState, const Eigen::VectorXd & baseSystemDisturbance, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorDisturbance) const {
	Eigen::VectorXd out = Eigen::VectorXd::Zero(3);
	double phis = baseSystemState(5) + sensorState(2);
	out(0) = Ts * (baseSystemState(0)*cos(phis) - baseSystemState(1)*sin(phis));
	out(1) = Ts * (baseSystemState(0)*sin(phis) + baseSystemState(1)*cos(phis));
	return out;
}

bool INSSensor::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<KUKAyouBot>(ptr) || _isCompatible<Truck>(ptr);
}

std::vector<std::string> INSSensor::getStateNames() const {
	return {"xs","ys","dphi"};
}
std::vector<std::string> INSSensor::getNoiseNames() const {
	return std::vector<std::string>();
}
std::vector<std::string> INSSensor::getDisturbanceNames() const {
	return { "wx","wy","wphi" };
}
std::vector<std::string> INSSensor::getOutputNames() const {
	return {"xs","ys","phis"};
}

std::string INSSensor::getName() const {
	return "INS";
}
