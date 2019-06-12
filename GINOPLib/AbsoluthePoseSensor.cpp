#include "AbsoluthePoseSensor.h"

AbsoluthePoseSensor::AbsoluthePoseSensor(BaseSystem::BaseSystemPtr ptr, bool withOrientation) :
	Sensor(ptr), withOrientation(withOrientation) {}

AbsoluthePoseSensor::~AbsoluthePoseSensor() {}

unsigned int AbsoluthePoseSensor::getNumOfStates() const { return 0; }

unsigned int AbsoluthePoseSensor::getNumOfDisturbances() const { return 0; }

unsigned int AbsoluthePoseSensor::getNumOfOutputs() const {
	if (withOrientation)
		return 3;
	else return 2;
}

unsigned int AbsoluthePoseSensor::getNumOfNoises() const {
	if (withOrientation)
		return 3;
	else return 2;
}

Eigen::MatrixXd AbsoluthePoseSensor::getA0(double Ts) const {
	return Eigen::MatrixXd(0, getNumOfBaseSystemStates());
}

Eigen::MatrixXd AbsoluthePoseSensor::getAi(double Ts) const {
	return Eigen::MatrixXd(0, 0);
}

Eigen::MatrixXd AbsoluthePoseSensor::getB0(double Ts) const {
	return Eigen::MatrixXd(0, getNumOfBaseSystemDisturbances());
}

Eigen::MatrixXd AbsoluthePoseSensor::getBi(double Ts) const {
	return Eigen::MatrixXd(0, 0);
}

Eigen::MatrixXd AbsoluthePoseSensor::getC0(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(getNumOfOutputs(), getNumOfBaseSystemStates());
	out(0, 3) = 1;
	out(1, 4) = 1;
	if (withOrientation)
		out(2, 5) = 1;
	return out;
}

Eigen::MatrixXd AbsoluthePoseSensor::getCi(double Ts) const {
	return Eigen::MatrixXd(getNumOfOutputs(), 0);
}

Eigen::MatrixXd AbsoluthePoseSensor::getD0(double Ts) const {
	return Eigen::MatrixXd::Zero(getNumOfOutputs(), getNumOfBaseSystemNoises());
}

Eigen::MatrixXd AbsoluthePoseSensor::getDi(double Ts) const {
	return Eigen::MatrixXd::Identity(getNumOfOutputs(), getNumOfNoises());
}

Eigen::MatrixXd AbsoluthePoseSensor::getPInvDi(double Ts) const {
	return Eigen::MatrixXd::Identity(getNumOfNoises(), getNumOfOutputs());
}

bool AbsoluthePoseSensor::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<youBotSystem>(ptr);
}

std::vector<std::string> AbsoluthePoseSensor::getStateNames() const {
	return {};
}

std::vector<std::string> AbsoluthePoseSensor::getNoiseNames() const {
	if (withOrientation)
		return { "vx","vy","vphi" };
	else return { "vx","vy" };
}

std::vector<std::string> AbsoluthePoseSensor::getDisturbanceNames() const {
	return {};
}

std::vector<std::string> AbsoluthePoseSensor::getOutputNames() const {
	if (withOrientation)
		return { "yx","yy","yphi" };
	else return { "yx","yy" };
}

std::string AbsoluthePoseSensor::getName() const {
	if (withOrientation)
		return "AbsoluthePoseSensor";
	else return "AbsoluthePositionSensor";
}
