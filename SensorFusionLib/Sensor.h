#pragma once

#include "BaseSystem.h"

class Sensor : public System
{
private:
	unsigned int numOfBaseSystemStates;
	unsigned int numOfBaseSystemDisturbances;
	unsigned int numOfBaseSystemNoises;

protected:
	template <class BaseSystemType>
	bool _isCompatible(BaseSystem::BaseSystemPtr ptr) const;

public:
	Sensor(BaseSystem::BaseSystemPtr ptr) : numOfBaseSystemStates(ptr->getNumOfStates()),
	numOfBaseSystemDisturbances(ptr->getNumOfDisturbances()), numOfBaseSystemNoises(ptr->getNumOfNoises()) {};

	unsigned int getNumOfBaseSystemStates() const;
	unsigned int getNumOfBaseSystemNoises() const;
	unsigned int getNumOfBaseSystemDisturbances() const;

	// The dynamics can be defined by the linear coefficient matrices and the nonlinearpart
	// x(k) = A0*xbase(k-1) + Ai*xsensor(k-1) + B0*wbase(k) + Bi*wsensor(k) + UpdateNonlinPart(xbase,xsensor,wbase,wsensor);
	// y(k) = C0*xbase(k) + Ci*xsensor(k) + D0*vbase(k) + Di*vsensor(k) + OutputNonlinPart(xbase,xsensor,vbase,vsensor);
	// but the dependencies of the nonlinearparts  on the x,w,v vectors must be elementwisely given
	virtual Eigen::MatrixXd getA0(double Ts) const = 0;
	virtual Eigen::MatrixXd getAi(double Ts) const = 0;

	virtual Eigen::MatrixXd getB0(double Ts) const = 0;
	virtual Eigen::MatrixXd getBi(double Ts) const = 0;

	virtual Eigen::MatrixXd getC0(double Ts) const = 0;
	virtual Eigen::MatrixXd getCi(double Ts) const = 0;

	virtual Eigen::MatrixXd getD0(double Ts) const = 0;
	virtual Eigen::MatrixXd getDi(double Ts) const = 0;


	// The nonlinear parts and the dependencies are by default zeros - the missing elements will be guessed zero
	virtual Eigen::VectorXi getUpdateNonlinearX0Dependencies() const;
	virtual Eigen::VectorXi getUpdateNonlinearXiDependencies() const;
	virtual Eigen::VectorXi getUpdateNonlinearW0Dependencies() const;
	virtual Eigen::VectorXi getUpdateNonlinearWiDependencies() const;

	virtual Eigen::VectorXi getOutputNonlinearX0Dependencies() const;
	virtual Eigen::VectorXi getOutputNonlinearXiDependencies() const;
	virtual Eigen::VectorXi getOutputNonlinearV0Dependencies() const;
	virtual Eigen::VectorXi getOutputNonlinearViDependencies() const;

	virtual Eigen::VectorXd UpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemDisturbance, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorDisturbance) const;

	virtual Eigen::VectorXd OutputNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemNoise, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorNoise) const;

	// The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions
	Eigen::VectorXd EvalUpdate(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemDisturbance,
		Eigen::VectorXd sensorState, Eigen::VectorXd sensorDisturbance) const;

	Eigen::VectorXd EvalOutput(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemNoise,
		Eigen::VectorXd sensorState, Eigen::VectorXd sensorNoise) const;

	virtual bool isCompatible(BaseSystem::BaseSystemPtr ptr) const = 0;

	Eigen::VectorXd genNonlinearPart(UpdateType type, double Ts, const Eigen::VectorXd& baseSystemState,
		const Eigen::VectorXd& baseSystemIn, const Eigen::VectorXd& sensorState,
		const Eigen::VectorXd& sensorIn) const;

	Eigen::VectorXi genNonlinearBaseSystemDependency(UpdateType outType, InputType inType) const;

	Eigen::VectorXi genNonlinearSensorDependency(UpdateType outType, InputType inType) const;

	typedef std::shared_ptr<Sensor> SensorPtr;

	void systemTest() const;
};

template<class BaseSystemType>
inline bool Sensor::_isCompatible(BaseSystem::BaseSystemPtr ptr) const {
	BaseSystemType* casted = dynamic_cast<BaseSystemType*>(ptr.get());
	if (casted) return true;
	else return false;
}
