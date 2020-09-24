#include "Sensor2DPosewDrift.h"

using namespace SF;

unsigned int SF::Sensor2DPosewDrift::getNumOfStates() const {
	return 3;
}

unsigned int SF::Sensor2DPosewDrift::getNumOfDisturbances() const {
	return 3;
}

unsigned int SF::Sensor2DPosewDrift::getNumOfOutputs() const {
	return 3;
}

unsigned int SF::Sensor2DPosewDrift::getNumOfNoises() const {
	return 3;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getAs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 5);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getAs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getBs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 2);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getBs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3)*Ts;
	return out;
}

Eigen::VectorXd SF::Sensor2DPosewDrift::EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd & baseSystemState, const Eigen::VectorXd & baseSystemDisturbance, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorDisturbance) const {
	static Eigen::VectorXd out = Eigen::VectorXd::Zero(3);
	out(0) = baseSystemState(3)*Ts*(cos(baseSystemState(2) + sensorState(2)) - cos(baseSystemState(2)));
	out(1) = baseSystemState(3)*Ts*(sin(baseSystemState(2) + sensorState(2)) - sin(baseSystemState(2)));
	return out;
}

Eigen::VectorXi SF::Sensor2DPosewDrift::getStateUpdateNonlinXbsDep() const {
	static Eigen::VectorXi out = Eigen::VectorXi::Zero(5);
	out(2) = 1;
	out(3) = 1;
	return out;
}

Eigen::VectorXi SF::Sensor2DPosewDrift::getStateUpdateNonlinXsDep() const {
	static Eigen::VectorXi out = Eigen::VectorXi::Zero(3);
	out(2) = 1;
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getCs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 5);
	for (int i = 0; i < 3; i++)
		out(i, i) = 1;
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getCs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getDs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 0);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewDrift::getDs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3);
	return out;
}

bool SF::Sensor2DPosewDrift::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<Vechicle2D>(ptr);
}

Eigen::VectorXi ZZO() {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(3);
	out(2) = 1;
	return out;
}

const Eigen::VectorXi & SF::Sensor2DPosewDrift::getIfOutputIsRad() const {
	static Eigen::VectorXi out = ZZO();
	return out;
}
