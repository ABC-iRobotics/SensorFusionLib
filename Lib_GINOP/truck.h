#pragma once

#include "BaseSystem.h"

/*! \brief Class that implements the properties of a Truck
*
* The dynamics:
*
* \f[
	\begin{bmatrix}
	v_x \\ v_y \\ \omega \\ x \\ y \\ \phi
	\end{bmatrix}_{k}
	=
	\begin{bmatrix}
		1-\frac{T_s}{T_d} & 0 & 0 & 0 & 0 & 0 \\
		0 & 1-\frac{T_s}{T_d} & 0 & 0 & 0 & 0 \\
		0 & 0 & 1-\frac{T_s}{T_d} & 0 & 0 & 0  \\
		0 & 0 & 0 &					1 & 0 & 0 \\
		0 & 0 & 0 &					0 & 1 & 0 \\
		0 & 0 & T_s &				0 & 0 & 1 
	\end{bmatrix}
	\begin{bmatrix}
	v_x \\ v_y \\ \omega \\ x \\ y \\ \phi 
	\end{bmatrix}_{k-1}
	+
	\begin{bmatrix}
	\frac{T_s}{T_d} \mathbf I_3   \\
	\mathbf 0  
	\end{bmatrix}
	\begin{bmatrix}
	v_x \\ v_y \\ \omega 
	\end{bmatrix}_{odo}
	+ 
	\begin{bmatrix}
	0 \\ 0 \\ 0 \\ T_sv_x\cos\phi - T_sv_y\sin\phi \\ T_sv_x\sin\phi + T_sv_y\cos\phi \\ 0
	\end{bmatrix}
\f]
* where \f$T_d\f$ is the assumed fastest time constant (\f$ T_d\geq T_s\f$)
* and for the Truck the odometry can br written as
* \f[
	\begin{bmatrix}
		v_x \\ v_y \\ \omega
	\end{bmatrix}
	=
	\begin{bmatrix}
	\frac{1}{2}r & 	\frac{1}{2}r \\
	0 & 0 \\
	-\frac{r}{w} & \frac{r}{w}
	\end{bmatrix}
	\begin{bmatrix}
	u_1 \\ u_2
	\end{bmatrix},
\f]
* The output equation has zero elements.
*
*/
class Truck : public BaseSystem {
private:
	const double dTdyn = 0.02;
	double dGeomW;
	double dGeomR;

public:
	Truck(double Tdyn, double W, double R, unsigned int ID); /*!< Constructor. Tdyn is the expected fastest time constant, W, R stand for the width of the platform and radius of its wheels. */

	unsigned int getNumOfDisturbances() const override;

	unsigned int getNumOfOutputs() const override;

	unsigned int getNumOfNoises() const override;

	unsigned int getNumOfStates() const override;

	Eigen::MatrixXd getA(double Ts) const override;

	Eigen::MatrixXd getB(double Ts) const override;

	Eigen::MatrixXd getPInvB(double Ts) const override;

	Eigen::MatrixXd getC(double Ts) const;

	Eigen::MatrixXd getD(double Ts) const;

	Eigen::VectorXi getStateUpdateNonlinXDep() const;

	Eigen::VectorXd EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd& state,
		const Eigen::VectorXd& disturbance) const override;

	std::vector<std::string> getStateNames() const override;

	std::vector<std::string> getNoiseNames() const override;

	std::vector<std::string> getDisturbanceNames() const override;

	std::vector<std::string> getOutputNames() const override;

	std::string getName() const override;
};

