#pragma once
#include "Sensor.h"

/*! \brief Class that implements the proprties of a 2D Inertial Measurement Unit (IMU)
*
* The raw measured acceleration and angular velocity are the output of this model
*
* The dynamics:
*
* \f[ \begin{bmatrix} v_{x,old}\\ v_{y,old} \\ s_x \\ s_y \\ s_\omega \end{bmatrix}_k
=
\begin{bmatrix} v_{x}\\ v_{y} \\ s_x \\ s_y \\ s_\omega \end{bmatrix}_{k-1}
\f]
*
* where \f$ v_{x,old} \f$, \f$ v_{y,old} \f$ are needed for deriving accelerations and \f$s_x \f$, \f$s_y\f$, \f$s_\omega\f$
* are the unknown scaling factors
*
* The output:
* 
* \f[ \begin{bmatrix} a_{x,meas} \\ a_{y,meas} \\ \omega_{meas} \end{bmatrix}
=
\begin{bmatrix}
 \left( (v_x - v_{x,old})/Ts - \omega\cdot v_y \right) s_x \\
 \left( (v_y - v_{y,old})/Ts + \omega\cdot v_x \right) s_y \\
 \omega \cdot s_{\omega}
\end{bmatrix} + \mathbf I \cdot \mathbf v_s
\f]
*/
class IMUSensor : public Sensor
{
public:
	IMUSensor(BaseSystem::BaseSystemPtr ptr, unsigned int ID); /*!< Constructor */

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

	Eigen::MatrixXd getPInvDs(double Ts) const override;

	Eigen::VectorXi getOutputUpdateNonlinXbsDep() const override;

	Eigen::VectorXi getOutputUpdateNonlinXsDep() const override;

	Eigen::VectorXd EvalOutputUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState, const Eigen::VectorXd& baseSystemNoise,
		const Eigen::VectorXd& sensorState, const Eigen::VectorXd& sensorNoise) const override;

	bool isCompatible(BaseSystem::BaseSystemPtr ptr) const override;

	typedef std::shared_ptr<IMUSensor> IMUSensorPtr;  /*!< Shared pointer type for the IMUSensor class */

	std::vector<std::string> getStateNames() const override;

	std::vector<std::string> getNoiseNames() const override;

	std::vector<std::string> getDisturbanceNames() const override;

	std::vector<std::string> getOutputNames() const override;

	std::string getName() const override;
};

