#include "Sensor2DPosewCalibration.h"

using namespace SF;

unsigned int SF::Sensor2DPosewCalibration::getNumOfStates() const {
	return 3;
}

unsigned int SF::Sensor2DPosewCalibration::getNumOfDisturbances() const {
	return 3;
}

unsigned int SF::Sensor2DPosewCalibration::getNumOfOutputs() const {
	return 3;
}

unsigned int SF::Sensor2DPosewCalibration::getNumOfNoises() const {
	return 3;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getAs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 5);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getAs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getBs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 2);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getBs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3)*Ts;
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getCs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 5);
	for (int i = 0; i < 3; i++)
		out(i, i) = 1;
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getCs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 3);
	out(2, 2) = 1;
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getDs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 0);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPosewCalibration::getDs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3);
	return out;
}

bool SF::Sensor2DPosewCalibration::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<Vechicle2D>(ptr);
}

Eigen::VectorXi ZZO2() {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(3);
	out(2) = 1;
	return out;
}

const Eigen::VectorXi & SF::Sensor2DPosewCalibration::getIfOutputIsRad() const {
	static Eigen::VectorXi out = ZZO2();
	return out;
}

Eigen::VectorXd SF::Sensor2DPosewCalibration::EvalOutputUpdateNonlinearPart(double Ts, const Eigen::VectorXd & baseSystemState, const Eigen::VectorXd & baseSystemNoise, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorNoise) const {
	double c = cos(baseSystemState(2)), s = sin(baseSystemState(2));
	Eigen::VectorXd out(3);
	out(0) = c * sensorState(0) - s * sensorState(1);
	out(1) = s * sensorState(0) + c * sensorState(1);
	out(2) = 0;
	return out;
}

Eigen::VectorXi SF::Sensor2DPosewCalibration::getOutputUpdateNonlinXbsDep() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(5);
	out(2) = 1;
	return out;
}

Eigen::VectorXi SF::Sensor2DPosewCalibration::getOutputUpdateNonlinXsDep() const {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(3);
	out(0) = 1;
	out(1) = 1;
	return out;
}
