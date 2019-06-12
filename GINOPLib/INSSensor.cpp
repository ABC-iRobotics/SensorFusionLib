#include "INSSensor.h"
#include "youBotSystem.h"

Eigen::MatrixXd INSSensor::getA0(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 7);
}

Eigen::MatrixXd INSSensor::getAi(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd INSSensor::getB0(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 4);
}

Eigen::MatrixXd INSSensor::getBi(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd INSSensor::getC0(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 7);
	out(2, 5) = 1;
	return out;
}

Eigen::MatrixXd INSSensor::getCi(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd INSSensor::getD0(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 0);
}

Eigen::MatrixXd INSSensor::getDi(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 0);
}

Eigen::VectorXi INSSensor::getUpdateNonlinearX0Dependencies() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(7);
	out(0) = 1; out(1) = 1; out(5) = 1;
	return out;
}

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

Eigen::VectorXi INSSensor::getUpdateNonlinearXiDependencies() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(3);
	out(2) = 1;
	return out;
}

Eigen::VectorXd INSSensor::UpdateNonlinearPart(double Ts, const Eigen::VectorXd & baseSystemState, const Eigen::VectorXd & baseSystemDisturbance, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorDisturbance) const {
	Eigen::VectorXd out = Eigen::VectorXd::Zero(3);
	double phis = baseSystemState(5) + sensorState(2);
	out(0) = Ts * (baseSystemState(0)*cos(phis) - baseSystemState(1)*sin(phis));
	out(1) = Ts * (baseSystemState(0)*sin(phis) + baseSystemState(1)*cos(phis));
	return out;
}

bool INSSensor::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<youBotSystem>(ptr);
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
