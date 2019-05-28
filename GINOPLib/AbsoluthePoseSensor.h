#pragma once
#include "Sensor.h"
#include "youBotSystem.h"

class AbsoluthePoseSensor : public Sensor {
	bool withOrientation;

public:
	AbsoluthePoseSensor(BaseSystem::BaseSystemPtr ptr, bool withOrientation = true);

	~AbsoluthePoseSensor();

	unsigned int getNumOfStates() const;

	unsigned int getNumOfDisturbances() const;

	unsigned int getNumOfOutputs() const;

	unsigned int getNumOfNoises() const;

	Eigen::MatrixXd getA0(double Ts) const;

	Eigen::MatrixXd getAi(double Ts) const;

	Eigen::MatrixXd getB0(double Ts) const;

	Eigen::MatrixXd getBi(double Ts) const;

	Eigen::MatrixXd getC0(double Ts) const;

	Eigen::MatrixXd getCi(double Ts) const;

	Eigen::MatrixXd getD0(double Ts) const;

	Eigen::MatrixXd getDi(double Ts) const;

	bool isCompatible(BaseSystem::BaseSystemPtr ptr) const override;

	typedef std::shared_ptr<AbsoluthePoseSensor> AbsoluthePoseSensorPtr;

	std::vector<std::string> getStateNames() const override;

	std::vector<std::string> getNoiseNames() const override;

	std::vector<std::string> getDisturbanceNames() const override;

	std::vector<std::string> getOutputNames() const override;

	std::string getName() const override;
};

