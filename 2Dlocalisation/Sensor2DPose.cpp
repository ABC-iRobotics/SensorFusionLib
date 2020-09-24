#include "Sensor2DPose.h"

using namespace SF;

unsigned int SF::Sensor2DPose::getNumOfStates() const {
	return 0;
}

unsigned int SF::Sensor2DPose::getNumOfDisturbances() const {
	return 0;
}

unsigned int SF::Sensor2DPose::getNumOfOutputs() const {
	return 3;
}

unsigned int SF::Sensor2DPose::getNumOfNoises() const {
	return 3;
}

Eigen::MatrixXd SF::Sensor2DPose::getAs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(0, 5);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPose::getAs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(0, 0);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPose::getBs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(0, 2);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPose::getBs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(0, 0);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPose::getCs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 5);
	for (int i = 0; i < 3; i++)
		out(i, i) = 1;
	return out;
}

Eigen::MatrixXd SF::Sensor2DPose::getCs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 0);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPose::getDs_bs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, 0);
	return out;
}

Eigen::MatrixXd SF::Sensor2DPose::getDs(double Ts) const {
	static Eigen::MatrixXd out = Eigen::MatrixXd::Identity(3, 3);
	return out;
}

bool SF::Sensor2DPose::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<Vechicle2D>(ptr);
}

Eigen::VectorXi ZZO1() {
	Eigen::VectorXi out = Eigen::VectorXi::Zero(3);
	out(2) = 1;
	return out;
}

const Eigen::VectorXi & SF::Sensor2DPose::getIfOutputIsRad() const {
	static Eigen::VectorXi out = ZZO1();
	return out;
}
