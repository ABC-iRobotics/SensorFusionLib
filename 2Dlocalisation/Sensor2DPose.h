#pragma once
#include "Vechicle2D.h"
#include "Sensor.h"

namespace SF {

	/*! \brief Simple position-orientation sensor model for the Vechicle2D 2D position-velocity model
	*
	* The ouput characteristics can simply be written as
	* \f[
		\mathbf y_{s,k+1} = \mathbf I^{3\times 5}\mathbf x_{bs,k} + \mathbf v_{s,k},
	\f]
	* but it is proposed to use self calibration see class Sensor2DPosewCalibration.
	*
	*
	* The class does not perform any computation, does not store state estimation or measurements. Only describes the dynamics and stores the user defined ID.
	*/
	class Sensor2DPose : public Sensor {
	public:
		Sensor2DPose(int ID) : Sensor(std::make_shared<Vechicle2D>(0), ID) {}  /*!< Constructor, argument: unique, user defined ID */

		unsigned int getNumOfStates() const;

		unsigned int getNumOfDisturbances() const;

		unsigned int getNumOfOutputs() const;

		unsigned int getNumOfNoises() const;

		Eigen::MatrixXd getAs_bs(double Ts) const;

		Eigen::MatrixXd getAs(double Ts) const;

		Eigen::MatrixXd getBs_bs(double Ts) const;

		Eigen::MatrixXd getBs(double Ts) const;

		Eigen::MatrixXd getCs_bs(double Ts) const;

		Eigen::MatrixXd getCs(double Ts) const;

		Eigen::MatrixXd getDs_bs(double Ts) const;

		Eigen::MatrixXd getDs(double Ts) const;

		bool isCompatible(BaseSystem::BaseSystemPtr ptr) const;

		const Eigen::VectorXi& getIfOutputIsRad() const override;
	};
}