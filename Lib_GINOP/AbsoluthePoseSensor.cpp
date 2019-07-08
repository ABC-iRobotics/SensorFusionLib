#include "AbsoluthePoseSensor.h"
#include "KUKAyouBot.h"
#include "truck.h"

AbsoluthePoseSensor::AbsoluthePoseSensor(BaseSystem::BaseSystemPtr ptr, unsigned int ID, bool withOrientation) :
	Sensor(ptr, ID), withOrientation(withOrientation) {}

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

Eigen::MatrixXd AbsoluthePoseSensor::getAs_bs(double Ts) const {
	return Eigen::MatrixXd(0, getNumOfBaseSystemStates());
}

Eigen::MatrixXd AbsoluthePoseSensor::getAs(double Ts) const {
	return Eigen::MatrixXd(0, 0);
}

Eigen::MatrixXd AbsoluthePoseSensor::getBs_bs(double Ts) const {
	return Eigen::MatrixXd(0, getNumOfBaseSystemDisturbances());
}

Eigen::MatrixXd AbsoluthePoseSensor::getBs(double Ts) const {
	return Eigen::MatrixXd(0, 0);
}

Eigen::MatrixXd AbsoluthePoseSensor::getCs_bs(double Ts) const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(getNumOfOutputs(), getNumOfBaseSystemStates());
	out(0, 3) = 1;
	out(1, 4) = 1;
	if (withOrientation)
		out(2, 5) = 1;
	return out;
}

Eigen::MatrixXd AbsoluthePoseSensor::getCs(double Ts) const {
	return Eigen::MatrixXd(getNumOfOutputs(), 0);
}

Eigen::MatrixXd AbsoluthePoseSensor::getDs_bs(double Ts) const {
	return Eigen::MatrixXd::Zero(getNumOfOutputs(), getNumOfBaseSystemNoises());
}

Eigen::MatrixXd AbsoluthePoseSensor::getDs(double Ts) const {
	return Eigen::MatrixXd::Identity(getNumOfOutputs(), getNumOfNoises());
}

Eigen::MatrixXd AbsoluthePoseSensor::getPInvDs(double Ts) const {
	return Eigen::MatrixXd::Identity(getNumOfNoises(), getNumOfOutputs());
}

bool AbsoluthePoseSensor::isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	return _isCompatible<KUKAyouBot>(ptr) || _isCompatible<Truck>(ptr);
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
