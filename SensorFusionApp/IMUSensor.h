#pragma once

#include "Sensor.h"


/* Update:
| vx_old | = | 1 0 0 0 0 0 0 | | vx  | + | 0 0 0 0 0 | | vx_old |
| vy_old |   | 0 1 0 0 0 0 0 | | vy  | + | 0 0 0 0 0 | | vy_old |
|   sx   |   | 0 0 0 0 0 0 0 | | om  | + | 0 0 1 0 0 | |   sx   |
|   sy   |   | 0 0 0 0 0 0 0 | | x   | + | 0 0 0 1 0 | |   sy   |
|   som  |   | 0 0 0 0 0 0 0 | | y   | + | 0 0 0 0 1 | |   som  |
                               | phi |
		            	  	   | null|
	Output:
| ax | = 0*x_base + 0*x_sensor + 0*v_base + I_3*v_sensor + | ((vx-vxold)/Ts - omega*vy) * sx |
| ay |         											   | ((vy-vyold)/Ts + omega*vx) * sy |
| om |    												   | omega * som |

*/


class IMUSensor : public Sensor
{
public:
	IMUSensor() {};

	unsigned int getNumOfStates() const;

	unsigned int getNumOfDisturbances() const;

	unsigned int getNumOfOutputs() const;

	unsigned int getNumOfNoises() const;

	StatisticValue getInitializationStates() const override;

	Eigen::MatrixXd getA0(double Ts) const;

	Eigen::MatrixXd getAi(double Ts) const;

	Eigen::MatrixXd getB0(double Ts) const;

	Eigen::MatrixXd getBi(double Ts) const;

	Eigen::MatrixXd getC0(double Ts) const;

	Eigen::MatrixXd getCi(double Ts) const;

	Eigen::MatrixXd getD0(double Ts) const;

	Eigen::MatrixXd getDi(double Ts) const;

	Eigen::VectorXi getOutputNonlinearX0Dependencies() const override;

	Eigen::VectorXi getOutputNonlinearXiDependencies() const override;

	Eigen::VectorXd OutputNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState, const Eigen::VectorXd& baseSystemNoise,
		const Eigen::VectorXd& sensorState, const Eigen::VectorXd& sensorNoise) const override;

	bool isCompatible(BaseSystem::BaseSystemPtr ptr) const override;

	typedef std::shared_ptr<IMUSensor> IMUSensorPtr;
};

