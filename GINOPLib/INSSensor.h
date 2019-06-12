#pragma once
#include "Sensor.h"

/* 
	Update:
|  xs  | = | 1 0 0 | |  xs  | + | Ts (vx*cos(phi+dphi) - vy*sin(phi+dphi)) | + w
|  ys  |   | 0 1 0 | |  ys  |   | Ts (vy*cos(phi+dphi) + vx*sin(phi+dphi)) |
| dphi |   | 0 0 1 | | dphi |   |                      0                   |

	Output:
|  xs  | = |  xs  | + |  0  | + v
|  ys  |   |  ys  |   |  0  |
| phis |   | dphi |   | phi |
*/


class INSSensor : public Sensor
{
public:
	INSSensor(BaseSystem::BaseSystemPtr ptr) : Sensor(ptr) {};

	unsigned int getNumOfStates() const;

	unsigned int getNumOfDisturbances() const;

	unsigned int getNumOfOutputs() const;

	unsigned int getNumOfNoises() const;

	Eigen::MatrixXd getA0(double Ts) const;

	Eigen::MatrixXd getAi(double Ts) const;

	Eigen::MatrixXd getB0(double Ts) const;

	Eigen::MatrixXd getBi(double Ts) const;

	Eigen::MatrixXd getPInvBi(double Ts) const override;

	Eigen::MatrixXd getC0(double Ts) const;

	Eigen::MatrixXd getCi(double Ts) const;

	Eigen::MatrixXd getD0(double Ts) const;

	Eigen::MatrixXd getDi(double Ts) const;

	Eigen::VectorXi getUpdateNonlinearX0Dependencies() const override;

	Eigen::VectorXi getUpdateNonlinearXiDependencies() const override;

	Eigen::VectorXd UpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemDisturbance, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorDisturbance) const override;

	bool isCompatible(BaseSystem::BaseSystemPtr ptr) const override;

	typedef std::shared_ptr<INSSensor> IMUSensorPtr;

	std::vector<std::string> getStateNames() const override;

	std::vector<std::string> getNoiseNames() const override;

	std::vector<std::string> getDisturbanceNames() const override;

	std::vector<std::string> getOutputNames() const override;

	std::string getName() const override;
};

