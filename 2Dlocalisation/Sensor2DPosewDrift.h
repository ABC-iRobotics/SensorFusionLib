#pragma once
#include "Vechicle2D.h"
#include "Sensor.h"

namespace SF {

	/*! \brief Position-orientation sensor model for the Vechicle2D 2D position-velocity model taking into account its drift
	*
	* The state variables are the position, orientation corrections as \f$\mathbf x_{s} = \begin{bmatrix}
		x_c & y_c & \phi_c \end{bmatrix}^T \f$
	*
	* The disturbances are their changes \f$\mathbf w_{bs} = \begin{bmatrix} w_{xc} & w_{yc} & w_{\phi c} \end{bmatrix}^T \f$
	*
	* Then the dynamics can be written as
	* \f[
		\mathbf x_{abs,k+1} = \mathbf x_{abs,k} + T_s \mathbf w_{abs}, \\
	\f]
	* and the output characteristics
	* \f[
		\mathbf y_{abs,k+1} = \mathbf I^{3\times 5}\mathbf x_{bs,k} + diag(0,0,1)\mathbf x_{abs,k} + \mathbf v_{abs,k} + f(T_s,\mathbf x_{abs},\mathbf x_{bs}),
	\f]
	* where
	* \f[
		f(T_s,\mathbf x_{abs},\mathbf x_{bs}) = T_s\begin{bmatrix}
		\cos(\phi)x_c - \sin(\phi)y_c \\
		\sin(\phi)x_c + \cos(\phi)y_c \\
		0
		\end{bmatrix}.
	\f]
	*
	* The class does not perform any computation, does not store state estimation or measurements. Only describes the dynamics and stores the user defined ID.
	*/
	class Sensor2DPosewDrift : public Sensor {
	public:
		Sensor2DPosewDrift(int ID) : Sensor(std::make_shared<Vechicle2D>(0), ID) {}

		unsigned int getNumOfStates() const;

		unsigned int getNumOfDisturbances() const;

		unsigned int getNumOfOutputs() const;

		unsigned int getNumOfNoises() const;

		Eigen::MatrixXd getAs_bs(double Ts) const;

		Eigen::MatrixXd getAs(double Ts) const;

		Eigen::MatrixXd getBs_bs(double Ts) const;

		Eigen::MatrixXd getBs(double Ts) const;

		Eigen::VectorXd EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
			const Eigen::VectorXd& baseSystemDisturbance, const Eigen::VectorXd& sensorState,
			const Eigen::VectorXd& sensorDisturbance) const;

		Eigen::VectorXi getStateUpdateNonlinXbsDep() const;
		
		Eigen::VectorXi getStateUpdateNonlinXsDep() const;
		
		Eigen::MatrixXd getCs_bs(double Ts) const;

		Eigen::MatrixXd getCs(double Ts) const;

		Eigen::MatrixXd getDs_bs(double Ts) const;

		Eigen::MatrixXd getDs(double Ts) const;

		bool isCompatible(BaseSystem::BaseSystemPtr ptr) const;

		const Eigen::VectorXi& getIfOutputIsRad() const override;
	};

}