#pragma once
#include "Vechicle2D.h"
#include "Sensor.h"

namespace SF {

	/*! \brief Position-orientation sensor model for the Vechicle2D 2D position-velocity model with correction mechanism of the relative pose
	*
	* The state variables are the position, orientation drift as \f$\mathbf x_{s} = \begin{bmatrix}
		Dx & Dy & D\phi \end{bmatrix}^T \f$, where $D\phi = \phi_o-\phi$, $Dx = x_o-x$, $Dy = y_o-y$.
	*
	* The disturbances are their changes \f$\mathbf w_{s} = \begin{bmatrix} w_{xc} & w_{yc} & w_{\phi c} \end{bmatrix}^T \f$
	*
	* Then the dynamics can be written as
	* \f[
		\mathbf x_{s,k+1} = \mathbf x_{s,k} + T_s \mathbf w_{s} + f(T_s,\mathbf x_{s},\mathbf x_{bs})
	\f]
	* where
	* \f[
		f(T_s,\mathbf x_{s},\mathbf x_{bs}) = T_sv\begin{bmatrix}
			\cos(\phi + D\phi)- \cos(\phi) \\
			\sin(\phi + D\phi)- \sin(\phi) \\
			0
			\end{bmatrix}.
	\f]
	*
	* and the output characteristics
	* \f[
		\mathbf y_{s,k+1} = \mathbf I^{3\times 5} \mathbf x_{bs,k} + \mathbf x_{s,k} + \mathbf v_{s,k}.
	\f]
	*
	* The class does not perform any computation, does not store state estimation or measurements. Only describes the dynamics and stores the user defined ID.
	*/
	class Sensor2DPosewCalibration : public Sensor {
	public:
		Sensor2DPosewCalibration(int ID) : Sensor(std::make_shared<Vechicle2D>(0), ID) {}  /*!< Constructor, argument: unique, user defined ID */

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

		Eigen::VectorXd EvalOutputUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
			const Eigen::VectorXd& baseSystemNoise, const Eigen::VectorXd& sensorState,
			const Eigen::VectorXd& sensorNoise) const override;

		Eigen::VectorXi getOutputUpdateNonlinXbsDep() const override;

		Eigen::VectorXi getOutputUpdateNonlinXsDep() const override;
	};
}