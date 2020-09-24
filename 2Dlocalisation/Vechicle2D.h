#pragma once
#include "BaseSystem.h"

//#define use_Ts2_coeffs

namespace SF {

	/*! \brief 2D position-velocity model of a vechicle with differential drive (e.g. a mobile robot) as a BaseSystem
	*
	* The state variables are the position, orientation, velocity values as \f$\mathbf x_{bs} = \begin{bmatrix}
		x & y & \phi & v & \omega \end{bmatrix}^T \f$
	*
	* The disturbances are the accelerations \f$\mathbf w_{bs} = \begin{bmatrix} w_a & w_\epsilon \end{bmatrix}^T \f$
	*
	* Then the dynamics can be written as
	* \f[
		\mathbf x_{bs, k+1} = \mathbf A(T_s) \mathbf x_{bs,k} + \mathbf B(T_s) \mathbf w_{bs,k} + f(T_s,\mathbf x_{bs}),
		\f]
	* where
	* \f[
		\mathbf A(T_s) = \begin{bmatrix}
	1 & 0 & 0 & 0 & 0 \\
	0 & 1 & 0 & 0 & 0 \\
	0 & 0 & 1 & 0 & T_s \\
	0 & 0 & 0 & 1 & 0 \\
	0 & 0 & 0 & 0 & 1 
	\end{bmatrix},\ \ 
	\mathbf B(T_s) = \begin{bmatrix}
	0 & 0 \\
	0 & 0 \\
	0 & 0 \\
	T_s & 0 \\
	0 & T_s 
	\end{bmatrix}, \ \
	f(T_s,\mathbf x_{bs}) = \begin{bmatrix}
	T_s\cdot v \cdot \cos(\phi) \\
	T_s\cdot v \cdot \sin(\phi) \\
	0\\0\\0
	\end{bmatrix}. 
		\f]
	*
	* with defined use_Ts2_coeffs flag, the dynamics
	* \f[
		\mathbf x_{bs, k+1} = \mathbf A(T_s) \mathbf x_{bs,k} + \mathbf B(T_s) \mathbf w_{bs,k} + f(T_s,\mathbf x_{bs},\mathbf w_{bs}),
		\f]
	* where
	* \f[
	\mathbf B(T_s) = \begin{bmatrix}
	0 & 0 \\
	0 & 0 \\
	0 & T_s^2/2 \\
	T_s & 0 \\
	0 & T_s
	\end{bmatrix}, \ \
	f(T_s,\mathbf x_{bs}) = \begin{bmatrix}
	(T_s\cdot v + T_s^2/2 w_a) \cdot \cos(\phi) \\
	(T_s\cdot v + T_s^2/2 w_a) \cdot \sin(\phi) \\
	0\\0\\0
	\end{bmatrix}.
		\f]
	*
	* The class does not perform any computation, does not store state estimation or measurements. Only describes the dynamics and stores the user defined ID.
	*/
	class Vechicle2D : public BaseSystem {
	public:
		Vechicle2D(unsigned int ID) : BaseSystem(ID) {}

		Eigen::MatrixXd getA(double Ts) const;

		Eigen::MatrixXd getB(double Ts) const;

		Eigen::VectorXd EvalStateUpdateNonlinearPart(double Ts,
			const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const;

		Eigen::VectorXi getStateUpdateNonlinXDep() const override;

#ifdef use_Ts2_coeffs
		Eigen::VectorXi getStateUpdateNonlinWDep() const override;
#endif

		Eigen::MatrixXd getC(double Ts) const;

		Eigen::MatrixXd getD(double Ts) const;

		unsigned int getNumOfStates() const;

		unsigned int getNumOfDisturbances() const;

		unsigned int getNumOfOutputs() const;

		unsigned int getNumOfNoises() const;

		const Eigen::VectorXi& getIfStateIsRad() const override;
	};
}