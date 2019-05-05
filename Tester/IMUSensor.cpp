#include "pch.h"
#include "IMUSensor.h"
#include "youBotSystem.h"

Eigen::MatrixXd IMUSensor::getA0(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(5, 7);
	out(0, 0) = 1;
	out(1, 1) = 1;
	return out;
}

Eigen::MatrixXd IMUSensor::getAi(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Identity(5, 5);
	out(0, 0) = 0;
	out(1, 1) = 0;
	return out;
}

Eigen::MatrixXd IMUSensor::getB0(double Ts) const {
	return Eigen::MatrixXd::Zero(5, 4);
}

Eigen::MatrixXd IMUSensor::getBi(double Ts) const {
	return Eigen::MatrixXd::Zero(5, 0);
}

Eigen::MatrixXd IMUSensor::getC0(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 7);
}

Eigen::MatrixXd IMUSensor::getCi(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 5);
}

Eigen::MatrixXd IMUSensor::getD0(double Ts) const {
	return Eigen::MatrixXd::Zero(3, 0);
}

Eigen::MatrixXd IMUSensor::getDi(double Ts) const {
	return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::VectorXi IMUSensor::getOutputNonlinearX0Dependencies() const {
	return Eigen::VectorXi::Ones(3);
}

unsigned int IMUSensor::getNumOfStates() const {
	return 5;
}

unsigned int IMUSensor::getNumOfDisturbances() const {
	return 0;
}

unsigned int IMUSensor::getNumOfOutputs() const {
	return 3;
}

unsigned int IMUSensor::getNumOfNoises() const {
	return 3;
}

Eigen::VectorXi IMUSensor::getOutputNonlinearXiDependencies() const {
	return Eigen::VectorXi::Ones(5);
}

Eigen::VectorXd IMUSensor::OutputNonlinearPart(double Ts, const Eigen::VectorXd & baseSystemState,
	const Eigen::VectorXd & baseSystemNoise, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorNoise) const {
	Eigen::VectorXd out(3);
	//std::cout << "x0:\n" << baseSystemState << "\nv0:\n" << baseSystemNoise << "\nxi:\n" << sensorState << "\nvi:\n" << sensorNoise << "\n";
	out(0) = ((baseSystemState(0) - sensorState(0)) / Ts - baseSystemState(2)*baseSystemState(1))*
		sensorState(2);
	// ax_imu = ((vy-vyold)/Ts + omega*vx) * sx + dist_vy
	out(1) = ((baseSystemState(1) - sensorState(1)) / Ts + baseSystemState(2)*baseSystemState(0))*
		sensorState(3);
	// omega_imu = omega * s_omega + dist_omega
	out(2) = baseSystemState(2) * sensorState(4);
	//std::cout << "x0:\n" << out << "\n";
	return out;
}

bool IMUSensor::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<youBotSystem>(ptr);
}

std::vector<std::string> IMUSensor::getStateNames() const {
	return {"vxold","vyold","sx","sy","sphi"};
}
std::vector<std::string> IMUSensor::getNoiseNames() const {
	return {"vax","vay","vomega"};
}
std::vector<std::string> IMUSensor::getDisturbanceNames() const {
	return {};
}
std::vector<std::string> IMUSensor::getOutputNames() const {
	return {"yax","yay","yomega"};
}