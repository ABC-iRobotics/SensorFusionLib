#pragma once
// The dynamics and output of the base system can depend on its own states and
// the corresponding  disturbances and measurement noises

#include "System.h"
#include <iostream>
#include <memory>

/*! \brief Abstract class to describe the dynamics and related properties of a autonomous system (e.g. mobile robot)
*
* The model must be defined by the linear coefficient \f$\mathbf A_{bs} \f$, \f$\mathbf B_{bs} \f$, \f$\mathbf C_{bs} \f$, \f$\mathbf D_{bs} \f$ matrices and the nonlinearpart:
* 
* \f[
	\mathbf x_{bs,k} = \mathbf A_{bs} \mathbf x_{bs,k-1} + \mathbf B_{bs} \mathbf w_{bs} + f_{bs}(\mathbf x_{bs,k-1}, \mathbf w_{bs}),
	* \f]
	*
	* \f[
	\mathbf y_{bs} = \mathbf C_{bs} \mathbf x_{bs,k} + \mathbf D_{bs} \mathbf v_{bs} + g_{bs}(\mathbf x_{bs,k-1}, \mathbf v_{bs}).
 \f]
*
* Furthermore, must define
* - number of state, output, noise, disturbance
* - if the nonlinear depends on the elements of the state, noise, disturbance vectors
*
* Optionally, 
* - the names of state, output, noise, disturbance variables can be given
* - name of the system can be specificated
* - fast implementation of pinv(B), pinv(D) can be implemented
*
* The class does not perform any computation, does not store state estimation or measurements. Only describes the dynamics and stores the user defined ID.
*/
class BaseSystem : public System {
public:
	BaseSystem(unsigned int ID); /*!< Constructor, argument: unique, user defined ID */

	virtual Eigen::MatrixXd getA(double Ts) const = 0; /*!< Returns \f$ \mathbf A_{bs} \f$ (see spec. of BaseSystem class) for the given sampling time[s] */

	virtual Eigen::MatrixXd getB(double Ts) const = 0; /*!< Returns \f$ \mathbf B_{bs} \f$ (see spec. of BaseSystem class) for the given sampling time[s] */
	
	virtual Eigen::MatrixXd getC(double Ts) const = 0; /*!< Returns \f$ \mathbf C_{bs} \f$ (see spec. of BaseSystem class) for the given sampling time[s] */
	
	virtual Eigen::MatrixXd getD(double Ts) const = 0; /*!< Returns \f$ \mathbf D_{bs} \f$ (see spec. of BaseSystem class) for the given sampling time[s] */


	// The nonlinear parts and the dependencies are by default zeros
	virtual Eigen::VectorXi getStateUpdateNonlinXDep() const; /*!< Returns if the \f$f()\f$ function depends on the elements of \f$\mathbf x_{bs} \f$. By default it returns zeros, in other cases must be overridden.*/ 

	virtual Eigen::VectorXi getStateUpdateNonlinWDep() const; /*!< Returns if the \f$f()\f$ function depends on the elements of \f$\mathbf w_{bs} \f$. By default it returns zeros, in other cases must be overridden.*/ 

	virtual Eigen::VectorXi getOutputUpdateNonlinXDep() const;/*!< Returns if the \f$g()\f$ function depends on the elements of \f$\mathbf x_{bs} \f$. By default it returns zeros, in other cases must be overridden.*/ 

	virtual Eigen::VectorXi getOutputUpdateNonlinVDep() const; /*!< Returns if the \f$g()\f$ function depends on the elements of \f$\mathbf v_{bs} \f$. By default it returns zeros, in other cases must be overridden.*/ 

	virtual Eigen::VectorXd EvalStateUpdateNonlinearPart(double Ts,
		const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const;
	/*!< Returns the value of \f$f()\f$ function according to the inputs. By default it is zero, in other cases must be overridden. */

	virtual Eigen::VectorXd EvalOutputUpdateNonlinearPart(double Ts,
		const Eigen::VectorXd& state, const Eigen::VectorXd& noise) const;
	/*!< Returns the value of \f$g()\f$ function according to the inputs. By default it is zero, in other cases must be overridden. */
	

	// The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions
	Eigen::VectorXd EvalStateUpdate(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& BaseSystemDisturbance) const;
		/*!< Returns the new state vector according to the arguments. */

	Eigen::VectorXd EvalOutputUpdate(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& BaseSystemNoise) const;
		/*!< Returns the output vector according to the arguments. */

	Eigen::VectorXd EvalNonlinearPart(TimeUpdateType type, double Ts,
		const Eigen::VectorXd& state, const Eigen::VectorXd& in) const;
	/*!< Returns the value of \f$f()\f$ or \f$g()\f$ function according to the inputs.*/

	Eigen::VectorXi getNonlinDep(TimeUpdateType outType, VariableType inType); /*!< General interface to get the dependency vectors. */

	typedef std::shared_ptr<BaseSystem> BaseSystemPtr; /*!< Shared pointer type for the BaseSystem class */

	void systemTest() const; /*!< To check the consistency of the defined functions */

	virtual Eigen::MatrixXd getPInvB(double Ts) const; /*!< Returns \f$pinv(\mathbf B) \f$. Can be overridden to decrease computational complexity.*/

	virtual Eigen::MatrixXd getPInvD(double Ts) const; /*!< Returns \f$pinv(\mathbf D) \f$. Can be overridden to decrease computational complexity.*/
};