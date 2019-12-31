#pragma once

#include "BaseSystem.h"

/*! \brief Abstract class to describe the dynamics and related properties of a sensor related to a given BaseSystem (e.g. IMU for KUKA youBot)
*
* Considering a BaseSystem dynamics:
*
* \f[
	\mathbf x_{bs,k} = \mathbf A_{bs} \mathbf x_{bs,k-1} + \mathbf B_{bs} \mathbf w_{bs} + f_{bs}(\mathbf x_{bs,k-1}, \mathbf w_{bs}),
	* \f]
	*
	* \f[
	\mathbf y_{bs} = \mathbf C_{bs} \mathbf x_{bs,k} + \mathbf D_{bs} \mathbf v_{bs} + g_{bs}(\mathbf x_{bs,k-1}, \mathbf v_{bs}).
 \f]
*
* The characteristics of a sensor can be given as
* \f[ \mathbf x_{s,k} = \mathbf A_{s/bs} \mathbf x_{bs,k-1} + \mathbf A_s \mathbf x_{s,k-1} + 
	\mathbf B_{s/bs} \mathbf w_{bs} + \mathbf B_s \mathbf w_{s} + f_s(\mathbf x_{bs,k-1},\mathbf x_{s,k-1}, \mathbf w_{bs},\mathbf w_{s}), 
	\f]
*
*	\f[ \mathbf y_{s} = \mathbf C_{s/bs} \mathbf x_{bs,k} + \mathbf C_s \mathbf x_{s,k} + 
	\mathbf D_{s/bs} \mathbf v_{bs} + \mathbf D_s \mathbf v_{s} + g_s(\mathbf x_{bs,k},\mathbf x_{s,k}, \mathbf v_{bs},\mathbf v_{s})
\f]
*
* The model must be defined by the linear coefficient \f$\mathbf A_{s/bs} \f$, \f$\mathbf B_{s/bs} \f$, \f$\mathbf C_{s/bs} \f$, \f$\mathbf D_{s/bs} \f$
* \f$\mathbf A_{s} \f$, \f$\mathbf B_{s} \f$, \f$\mathbf C_{s} \f$, \f$\mathbf D_{s} \f$matrices and the nonlinearparts: functions \f$f_s() \f$ and \f$g_s() \f$
* Furthermore, must define
* - number of state, output, noise, disturbance related the sensor
* - if the nonlinear depends on the elements of the state, noise, disturbance vectors related to the sensor and the BaseSystem
*
* Optionally,
* - the names of state, output, noise, disturbance variables (of the sensor) can be given
* - name of the sensor can be specificated
* - fast implementation of pinv(B), pinv(D) can be implemented
*
* The class does not perform any computation, does not store state estimation or measurements. Only describes the dynamics and stores the user defined ID.
*/
class Sensor : public System {
private:
	unsigned int numOfBaseSystemStates;   /*!< Number of states of the basesystem */
	unsigned int numOfBaseSystemDisturbances; /*!< Number of disturbances of the basesystem */
	unsigned int numOfBaseSystemNoises; /*!< Number of noises of the basesystem */

protected:
	template <class BaseSystemType>
	bool _isCompatible(BaseSystem::BaseSystemPtr ptr) const; /*!< Template function to implement isCompatible() function*/

public:
	Sensor(BaseSystem::BaseSystemPtr ptr, unsigned int ID); /*!< Constructor */

	unsigned int getNumOfBaseSystemStates() const; /*!< Returns the number of state variables of the related BaseSystem */
	unsigned int getNumOfBaseSystemNoises() const;/*!< Returns the number of noise signals of the related BaseSystem */
	unsigned int getNumOfBaseSystemDisturbances() const; /*!< Returns the number of disturbance signals of the related BaseSystem */

	virtual Eigen::MatrixXd getAs_bs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf A_{s,bs}\f$ according to the sampling time argument */
	virtual Eigen::MatrixXd getAs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf A_{s}\f$ according to the sampling time argument */

	virtual Eigen::MatrixXd getBs_bs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf B_{s,bs}\f$ according to the sampling time argument */
	virtual Eigen::MatrixXd getBs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf B_{s}\f$ according to the sampling time argument */

	virtual Eigen::MatrixXd getCs_bs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf C_{s,bs}\f$ according to the sampling time argument */
	virtual Eigen::MatrixXd getCs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf C_{s}\f$ according to the sampling time argument */

	virtual Eigen::MatrixXd getDs_bs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf D_{s,bs}\f$ according to the sampling time argument */
	virtual Eigen::MatrixXd getDs(double Ts) const = 0; /*!< Returns the matrix \f$\mathbf D_{s}\f$ according to the sampling time argument */


	// The nonlinear parts and the dependencies are by default zeros - the missing elements will be guessed zero
	virtual Eigen::VectorXi getStateUpdateNonlinXbsDep() const; /*!< Returns if the \f$f()\f$ function depends on the elements of \f$\mathbf x_{bs} \f$. By default it returns zeros, in other cases must be overridden.  */ 
	virtual Eigen::VectorXi getStateUpdateNonlinXsDep() const; /*!< Returns if the \f$f()\f$ function depends on the elements of \f$\mathbf x_{s} \f$. By default it returns zeros, in other cases must be overridden. */ 
	virtual Eigen::VectorXi getStateUpdateNonlinWbsDep() const; /*!< Returns if the \f$f()\f$ function depends on the elements of \f$\mathbf w_{bs} \f$. By default it returns zeros, in other cases must be overridden. */ 
	virtual Eigen::VectorXi getStateUpdateNonlinWsDep() const; /*!< Returns if the \f$f()\f$ function depends on the elements of \f$\mathbf w_{s} \f$. By default it returns zeros, in other cases must be overridden. */ 

	virtual Eigen::VectorXi getOutputUpdateNonlinXbsDep() const; /*!< Returns if the \f$g()\f$ function depends on the elements of \f$\mathbf x_{bs} \f$. By default it returns zeros, in other cases must be overridden. */ 
	virtual Eigen::VectorXi getOutputUpdateNonlinXsDep() const; /*!< Returns if the \f$g()\f$ function depends on the elements of \f$\mathbf x_{s} \f$. By default it returns zeros, in other cases must be overridden. */
	virtual Eigen::VectorXi getOutputUpdateNonlinVbsDep() const; /*!< Returns if the \f$g()\f$ function depends on the elements of \f$\mathbf v_{bs} \f$. By default it returns zeros, in other cases must be overridden. */
	virtual Eigen::VectorXi getOutputUpdateNonlinVsDep() const; /*!< Returns if the \f$g()\f$ function depends on the elements of \f$\mathbf v_{s} \f$. By default it returns zeros, in other cases must be overridden. */

	virtual Eigen::VectorXd EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemDisturbance, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorDisturbance) const; /*!< Returns the value of \f$f()\f$ function according to the inputs. By default it is zero, in other cases must be overridden. */

	virtual Eigen::VectorXd EvalOutputUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemNoise, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorNoise) const; /*!< Returns the value of \f$g()\f$ function according to the inputs. By default it is zero, in other cases must be overridden. */

	// The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions
	Eigen::VectorXd EvalStateUpdate(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemDisturbance,
		Eigen::VectorXd sensorState, Eigen::VectorXd sensorDisturbance) const;
		/*!< Returns the new sensor state vector according to the arguments. */

	Eigen::VectorXd EvalOutputUpdate(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemNoise,
		Eigen::VectorXd sensorState, Eigen::VectorXd sensorNoise) const;
		/*!< Returns the sensor output vector according to the arguments. */

	virtual bool isCompatible(BaseSystem::BaseSystemPtr ptr) const = 0; /*!< To check if the sensor is compatible with a BaseSystem.  (It must be implement using the _isCompatible<class> function.) */

	Eigen::VectorXd EvalNonlinearPart(TimeUpdateType type, double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemIn, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorIn) const; /*!< Returns the value of \f$f()\f$ or \f$g()\f$ function according to the inputs. */

	Eigen::VectorXi getNonlinDepOnBaseSystemSignals(TimeUpdateType outType, VariableType inType) const;  /*!< General interface to get the dependency vectors. */

	Eigen::VectorXi getNonlinDepOnSensorSignals(TimeUpdateType outType, VariableType inType) const;  /*!< General interface to get the dependency vectors. */

	typedef std::shared_ptr<Sensor> SensorPtr; /*!< Shared pointer type for the Sensor class */

	void systemTest() const; /*!< To check the consistency of the defined functions */

	virtual Eigen::MatrixXd getPInvBs(double Ts) const; /*!< Returns \f$pinv(\mathbf B_s) \f$. Can be overridden to decrease computational complexity.*/

	virtual Eigen::MatrixXd getPInvDs(double Ts) const; /*!< Returns \f$pinv(\mathbf D_s) \f$. Can be overridden to decrease computational complexity.*/
};

template<class BaseSystemType>
inline bool Sensor::_isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	BaseSystemType* casted = dynamic_cast<BaseSystemType*>(ptr.get());
	if (casted) return true;
	else return false;
}
