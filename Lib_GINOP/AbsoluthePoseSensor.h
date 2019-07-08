#pragma once
#include "Sensor.h"

/*! \brief Class that implements the model of a sensor that provides 2D position and - optionally - orientation information
* with random noise
*
* It can be to model GPS or other similar sensors...
*
* It does not have state variables, the dynamics equation has zero elements.
*
* The output:
*
* \f[ \begin{bmatrix} x_{meas} \\ y_{meas} \\ \varphi_{meas} \end{bmatrix}
=
 \begin{bmatrix} x \\ y \\ \varphi \end{bmatrix} + \mathbf I_3 \cdot \mathbf v_s
\f]
*
* or
* \f[ \begin{bmatrix} x_{meas} \\ y_{meas}  \end{bmatrix}
=
 \begin{bmatrix} x \\ y  \end{bmatrix} + \mathbf I_2 \cdot \mathbf v_s
\f]
* according to the "withOrientation" flag
*/
class AbsoluthePoseSensor : public Sensor {
	bool withOrientation;

public:
	AbsoluthePoseSensor(BaseSystem::BaseSystemPtr ptr,
		unsigned int ID, bool withOrientation = true); /*!< Constructor. If the modelled sensor does not provide information about orientation, use "withOrientation=false" argument.*/

	~AbsoluthePoseSensor();

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

	bool isCompatible(BaseSystem::BaseSystemPtr ptr) const override;

	typedef std::shared_ptr<AbsoluthePoseSensor> AbsoluthePoseSensorPtr;  /*!< Shared pointer type for the IMUSensor class */

	std::vector<std::string> getStateNames() const override;

	std::vector<std::string> getNoiseNames() const override;

	std::vector<std::string> getDisturbanceNames() const override;

	std::vector<std::string> getOutputNames() const override;

	std::string getName() const override;
};

