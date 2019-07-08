#pragma once
#include "Sensor.h"

/*! \brief Class that implements the proprties of a 2D Inertial Navigation System (INS)
*
* It integrates the measured acceleration and angular velocity to obtain position and orientation
*
* States:
* - \f$ \Delta\phi \f$ error of the orientation
* - \f$ x_s\f$, \f$ y_s\f$ integrated position using the wrong orientation value
*
* Because the BaseSystem states were: \f$ \begin{bmatrix} v_x & v_y & \omega & x & y & \phi & ... \end{bmatrix}^T\f$ the dynamics can be written as
*
* \f[ 
\begin{bmatrix}
	x_s \\ y_s \\ \Delta \phi
	\end{bmatrix}_k
	=
	\mathbf I
	\begin{bmatrix}
	x_s \\ y_s \\ \Delta \phi
	\end{bmatrix}_{k-1}
	+
	\mathbf I
	\mathbf w_s
	+
	\begin{bmatrix}
	T_s v_x \cos(\phi+\Delta\phi) - T_s v_y \sin(\phi+\Delta\phi) \\
	T_s v_x \sin(\phi+\Delta\phi) + T_s v_y \cos(\phi+\Delta\phi) \\
	0
	\end{bmatrix}
\f]
*
* The output equation:
*
*\f[
\begin{bmatrix}
	x_s \\ y_s \\ \Delta \phi
	\end{bmatrix}
	=
	\mathbf I
	\begin{bmatrix}
	x_s \\ y_s \\ \Delta \phi
	\end{bmatrix}
\f]
*/
class INSSensor : public Sensor
{
public:
	INSSensor(BaseSystem::BaseSystemPtr ptr, unsigned int ID); /*!< Constructor */

	unsigned int getNumOfStates() const;

	unsigned int getNumOfDisturbances() const;

	unsigned int getNumOfOutputs() const;

	unsigned int getNumOfNoises() const;

	Eigen::MatrixXd getAs_bs(double Ts) const;

	Eigen::MatrixXd getAs(double Ts) const;

	Eigen::MatrixXd getBs_bs(double Ts) const;

	Eigen::MatrixXd getBs(double Ts) const;

	Eigen::MatrixXd getPInvBs(double Ts) const override;

	Eigen::MatrixXd getCs_bs(double Ts) const;

	Eigen::MatrixXd getCs(double Ts) const;

	Eigen::MatrixXd getDs_bs(double Ts) const;

	Eigen::MatrixXd getDs(double Ts) const;

	Eigen::VectorXi getStateUpdateNonlinXbsDep() const override;

	Eigen::VectorXi getStateUpdateNonlinXsDep() const override;

	Eigen::VectorXd EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemDisturbance, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorDisturbance) const override;

	bool isCompatible(BaseSystem::BaseSystemPtr ptr) const override;

	typedef std::shared_ptr<INSSensor> INSSensorPtr; /*!< Shared pointer type for the INSSensor class */

	std::vector<std::string> getStateNames() const override;

	std::vector<std::string> getNoiseNames() const override;

	std::vector<std::string> getDisturbanceNames() const override;

	std::vector<std::string> getOutputNames() const override;

	std::string getName() const override;
};

