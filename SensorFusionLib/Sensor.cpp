#include "pch.h"
#include "Sensor.h"

// The nonlinear parts and the dependencies are by default zeros

Eigen::VectorXi Sensor::getUpdateNonlinearX0Dependencies() const {
	return Eigen::VectorXi::Zero(getNumOfBaseSystemStates());
}

Eigen::VectorXi Sensor::getUpdateNonlinearXiDependencies() const {
	return Eigen::VectorXi::Zero(getNumOfStates());
}

Eigen::VectorXi Sensor::getUpdateNonlinearW0Dependencies() const {
	return Eigen::VectorXi::Zero(getNumOfBaseSystemDisturbances());
}

 Eigen::VectorXi Sensor::getUpdateNonlinearWiDependencies() const {
	return Eigen::VectorXi::Zero(getNumOfDisturbances());
}

 Eigen::VectorXi Sensor::getOutputNonlinearX0Dependencies() const {
	 return Eigen::VectorXi::Zero(getNumOfBaseSystemStates());
 }

 Eigen::VectorXi Sensor::getOutputNonlinearXiDependencies() const {
	 return Eigen::VectorXi::Zero(getNumOfStates());
 }

 Eigen::VectorXi Sensor::getOutputNonlinearV0Dependencies() const {
	 return Eigen::VectorXi::Zero(getNumOfBaseSystemNoises());
 }

 Eigen::VectorXi Sensor::getOutputNonlinearViDependencies() const {
	 return Eigen::VectorXi::Zero(getNumOfNoises());
 }

 Eigen::VectorXd Sensor::UpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
	 const Eigen::VectorXd& baseSystemDisturbance, const Eigen::VectorXd& sensorState,
	 const Eigen::VectorXd& sensorDisturbance) const {
	 return Eigen::VectorXd::Zero(getNumOfStates());
 }

 Eigen::VectorXd Sensor::OutputNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
	 const Eigen::VectorXd& baseSystemNoise, const Eigen::VectorXd& sensorState,
	 const Eigen::VectorXd& sensorNoise) const {
	 return Eigen::VectorXd::Zero(getNumOfOutputs());
 }

 Function4 Sensor::getUpdateMapping(double Ts) const {
	 return Function4(getA0(Ts), getB0(Ts), getAi(Ts), getBi(Ts),
		 getUpdateNonlinearX0Dependencies(), getUpdateNonlinearW0Dependencies(),
		 getUpdateNonlinearXiDependencies(), getUpdateNonlinearWiDependencies(),
		 [this, Ts](const Eigen::VectorXd& baseSystemState, const Eigen::VectorXd& baseSystemDisturbance,
			 const Eigen::VectorXd& sensorState, const Eigen::VectorXd& sensorDisturbance) {
		 return this->UpdateNonlinearPart(Ts, std::move(baseSystemState), std::move(baseSystemDisturbance),
			 std::move(sensorState), std::move(sensorDisturbance)); });
 }

 Function4 Sensor::getOutputMapping(double Ts, bool empty) const {
	 if (empty)
		 return Function4(Eigen::MatrixXd(0, numOfBaseSystemStates), Eigen::MatrixXd(0, numOfBaseSystemNoises),
			 Eigen::MatrixXd(0, getNumOfStates()), Eigen::MatrixXd(0, 0),
			 Eigen::VectorXi::Zero(0), Eigen::VectorXi::Zero(0),
			 Eigen::VectorXi::Zero(0), Eigen::VectorXi::Zero(0),
			 [](const Eigen::VectorXd& baseState, const Eigen::VectorXd& baseDisturbance,
				 const Eigen::VectorXd& sensorState, const Eigen::VectorXd& sensorDisturbance) {
			 return Eigen::VectorXd(0); });
	 else
		 return Function4(getC0(Ts), getD0(Ts), getCi(Ts), getDi(Ts),
			 getOutputNonlinearX0Dependencies(), getOutputNonlinearV0Dependencies(),
			 getOutputNonlinearXiDependencies(), getOutputNonlinearViDependencies(),
			 [this, Ts](const Eigen::VectorXd& baseSystemState, const Eigen::VectorXd& baseSystemNoise,
				 const Eigen::VectorXd& sensorState, const Eigen::VectorXd& sensorNoise) {
			 return this->OutputNonlinearPart(Ts, std::move(baseSystemState), std::move(baseSystemNoise),
				 std::move(sensorState), std::move(sensorNoise)); });
 }

 // The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions

 Eigen::VectorXd Sensor::EvalUpdate(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemDisturbance, Eigen::VectorXd sensorState, Eigen::VectorXd sensorDisturbance) const {
	 return getA0(Ts)*baseSystemState + getAi(Ts)*sensorState +
		 getB0(Ts)*baseSystemDisturbance + getBi(Ts)*sensorDisturbance +
		 UpdateNonlinearPart(Ts, baseSystemState, baseSystemDisturbance, sensorState, sensorDisturbance);
 }

 Eigen::VectorXd Sensor::EvalOutput(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemNoise, Eigen::VectorXd sensorState, Eigen::VectorXd sensorNoise) const {
	 return getC0(Ts)*baseSystemState + getCi(Ts)*sensorState +
		 getD0(Ts)*baseSystemNoise + getDi(Ts)*sensorNoise +
		 OutputNonlinearPart(Ts, baseSystemState, baseSystemNoise, sensorState, sensorNoise);
 }

 Eigen::VectorXd Sensor::genNonlinearPart(UpdateType type, double Ts, const Eigen::VectorXd & baseSystemState, const Eigen::VectorXd & baseSystemIn, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorIn) const {
	 switch (type) {
	 case TIMEUPDATE:
		 return UpdateNonlinearPart(Ts, baseSystemState, baseSystemIn, sensorState, sensorIn);
	 case MEASUREMENTUPDATE:
		 return OutputNonlinearPart(Ts, baseSystemState, baseSystemIn, sensorState, sensorIn);
	 }
 }

 Eigen::VectorXi Sensor::genNonlinearBaseSystemDependency(UpdateType outType, InputType inType) {
	 switch (outType) {
	 case TIMEUPDATE:
		 switch (inType) {
		 case STATE:
			 return getUpdateNonlinearX0Dependencies();
		 case INPUT:
			 return getUpdateNonlinearW0Dependencies();
		 }
	 case MEASUREMENTUPDATE:
		 switch (inType) {
		 case STATE:
			 return getOutputNonlinearX0Dependencies();
		 case INPUT:
			 return getOutputNonlinearV0Dependencies();
		 }
	 }
 }

 Eigen::VectorXi Sensor::genNonlinearSensorDependency(UpdateType outType, InputType inType) {
	 switch (outType) {
	 case TIMEUPDATE:
		 switch (inType) {
		 case InputType::STATE:
			 return getUpdateNonlinearXiDependencies();
		 case InputType::INPUT:
			 return getUpdateNonlinearWiDependencies();
		 }
	 case MEASUREMENTUPDATE:
		 switch (inType) {
		 case InputType::STATE:
			 return getOutputNonlinearXiDependencies();
		 case InputType::INPUT:
			 return getOutputNonlinearViDependencies();
		 }
	 }
 }

 unsigned int Sensor::getNumOfBaseSystemStates() const { return numOfBaseSystemStates; }

 unsigned int Sensor::getNumOfBaseSystemNoises() const { return numOfBaseSystemNoises; }

 unsigned int Sensor::getNumOfBaseSystemDisturbances() const {
	 return numOfBaseSystemDisturbances;
 }
