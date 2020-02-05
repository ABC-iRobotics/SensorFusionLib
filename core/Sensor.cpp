#include "Sensor.h"
#include "pinv.h"

using namespace SF;

// The nonlinear parts and the dependencies are by default zeros

Eigen::VectorXi Sensor::getStateUpdateNonlinXbsDep() const {
	return Eigen::VectorXi::Zero(getNumOfBaseSystemStates());
}

Eigen::VectorXi Sensor::getStateUpdateNonlinXsDep() const {
	return Eigen::VectorXi::Zero(getNumOfStates());
}

Eigen::VectorXi Sensor::getStateUpdateNonlinWbsDep() const {
	return Eigen::VectorXi::Zero(getNumOfBaseSystemDisturbances());
}

 Eigen::VectorXi Sensor::getStateUpdateNonlinWsDep() const {
	return Eigen::VectorXi::Zero(getNumOfDisturbances());
}

 Eigen::VectorXi Sensor::getOutputUpdateNonlinXbsDep() const {
	 return Eigen::VectorXi::Zero(getNumOfBaseSystemStates());
 }

 Eigen::VectorXi Sensor::getOutputUpdateNonlinXsDep() const {
	 return Eigen::VectorXi::Zero(getNumOfStates());
 }

 Eigen::VectorXi Sensor::getOutputUpdateNonlinVbsDep() const {
	 return Eigen::VectorXi::Zero(getNumOfBaseSystemNoises());
 }

 Eigen::VectorXi Sensor::getOutputUpdateNonlinVsDep() const {
	 return Eigen::VectorXi::Zero(getNumOfNoises());
 }

 Eigen::VectorXd Sensor::EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
	 const Eigen::VectorXd& baseSystemDisturbance, const Eigen::VectorXd& sensorState,
	 const Eigen::VectorXd& sensorDisturbance) const {
	 return Eigen::VectorXd::Zero(getNumOfStates());
 }

 Eigen::VectorXd Sensor::EvalOutputUpdateNonlinearPart(double Ts, const Eigen::VectorXd& baseSystemState,
	 const Eigen::VectorXd& baseSystemNoise, const Eigen::VectorXd& sensorState,
	 const Eigen::VectorXd& sensorNoise) const {
	 return Eigen::VectorXd::Zero(getNumOfOutputs());
 }

 // The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions

 Eigen::VectorXd Sensor::EvalStateUpdate(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemDisturbance, Eigen::VectorXd sensorState, Eigen::VectorXd sensorDisturbance) const {
	 return getAs_bs(Ts)*baseSystemState + getAs(Ts)*sensorState +
		 getBs_bs(Ts)*baseSystemDisturbance + getBs(Ts)*sensorDisturbance +
		 EvalStateUpdateNonlinearPart(Ts, baseSystemState, baseSystemDisturbance, sensorState, sensorDisturbance);
 }

 Eigen::VectorXd Sensor::EvalOutputUpdate(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemNoise,
	 Eigen::VectorXd sensorState, Eigen::VectorXd sensorNoise) const {
	 return getCs_bs(Ts)*baseSystemState + getCs(Ts)*sensorState +
		 getDs_bs(Ts)*baseSystemNoise + getDs(Ts)*sensorNoise +
		 EvalOutputUpdateNonlinearPart(Ts, baseSystemState, baseSystemNoise, sensorState, sensorNoise);
 }

 Eigen::VectorXd Sensor::EvalNonlinearPart(TimeUpdateType type, double Ts, const Eigen::VectorXd & baseSystemState,
	 const Eigen::VectorXd & baseSystemIn, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorIn) const {
	 switch (type) {
	 case STATE_UPDATE:
		 return EvalStateUpdateNonlinearPart(Ts, baseSystemState, baseSystemIn, sensorState, sensorIn);
	 case OUTPUT_UPDATE:
		 return EvalOutputUpdateNonlinearPart(Ts, baseSystemState, baseSystemIn, sensorState, sensorIn);
	 }
	 throw std::runtime_error(std::string("Sensor::EvalNonlinearPart(): Unknown input!"));
 }

 Eigen::VectorXi Sensor::getNonlinDepOnBaseSystemSignals(TimeUpdateType outType, VariableType inType) const {
	 switch (outType) {
	 case STATE_UPDATE:
		 switch (inType) {
		 case VAR_STATE:
			 return getStateUpdateNonlinXbsDep();
		 case VAR_EXTERNAL:
			 return getStateUpdateNonlinWbsDep();
		 }
	 case OUTPUT_UPDATE:
		 switch (inType) {
		 case VAR_STATE:
			 return getOutputUpdateNonlinXbsDep();
		 case VAR_EXTERNAL:
			 return getOutputUpdateNonlinVbsDep();
		 }
	 }
	 throw std::runtime_error(std::string("Sensor::getNonlinDepOnBaseSystemSignals(): Unknown input!"));
 }

 Eigen::VectorXi Sensor::getNonlinDepOnSensorSignals(TimeUpdateType outType, VariableType inType) const {
	 switch (outType) {
	 case STATE_UPDATE:
		 switch (inType) {
		 case VAR_STATE:
			 return getStateUpdateNonlinXsDep();
		 case VAR_EXTERNAL:
			 return getStateUpdateNonlinWsDep();
		 }
	 case OUTPUT_UPDATE:
		 switch (inType) {
		 case VAR_STATE:
			 return getOutputUpdateNonlinXsDep();
		 case VAR_EXTERNAL:
			 return getOutputUpdateNonlinVsDep();
		 }
	 }
	 throw std::runtime_error(std::string("Sensor::getNonlinDepOnSensorSignals(): Unknown input!"));
 }

 void Sensor::systemTest() const {
	 _systemTest();
	 int nx = getNumOfStates();
	 int ny = getNumOfOutputs();
	 int nv = getNumOfNoises();
	 int nw = getNumOfDisturbances();
	 int nx0 = getNumOfBaseSystemStates();
	 int nv0 = getNumOfBaseSystemNoises();
	 int nw0 = getNumOfBaseSystemDisturbances();
	 // Check A,B,C,D matrices
	 Eigen::MatrixXd M;
	 M = getAs(1);
	 if (M.cols() != nx || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getBs(1);
	 if (M.cols() != nw || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getCs(1);
	 if (M.cols() != nx || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getDs(1);
	 if (M.cols() != nv || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 //
	 M = getAs_bs(1);
	 if (M.cols() != nx0 || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getBs_bs(1);
	 if (M.cols() != nw0 || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getCs_bs(1);
	 if (M.cols() != nx0 || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getDs_bs(1);
	 if (M.cols() != nv0 || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 // Check pinv-s
	 if (!eq(getPInvBs(0.01),pinv(getBs(0.01))))
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 if (!eq(getPInvDs(0.01),pinv(getDs(0.01))))
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 // Check dependency vectors
	 Eigen::VectorXi v;
	 v = getStateUpdateNonlinXbsDep();
	 if (v.size() != nx0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getStateUpdateNonlinWbsDep();
	 if (v.size() != nw0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputUpdateNonlinXbsDep();
	 if (v.size() != nx0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputUpdateNonlinVbsDep();
	 if (v.size() != nv0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 //
	 v = getStateUpdateNonlinXsDep();
	 if (v.size() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getStateUpdateNonlinWsDep();
	 if (v.size() != nw)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputUpdateNonlinXsDep();
	 if (v.size() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputUpdateNonlinVsDep();
	 if (v.size() != nv)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 // Check nonlinear parts
	 Eigen::VectorXd m;
	 Eigen::VectorXd x = Eigen::VectorXd::Zero(nx);
	 Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx0);
	 Eigen::VectorXd i = Eigen::VectorXd::Zero(nw);
	 Eigen::VectorXd i0 = Eigen::VectorXd::Zero(nw0);
	 m = EvalStateUpdateNonlinearPart(1, x0, i0, x, i);
	 if (m.size() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 i0 = Eigen::VectorXd::Zero(nv0);
	 i = Eigen::VectorXd::Zero(nv);
	 m = EvalOutputUpdateNonlinearPart(1, x0, i0, x, i);
	 if (m.size() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
 }

 Eigen::MatrixXd Sensor::getPInvBs(double Ts) const {
	 return pinv(getBs(Ts));
 }

 Eigen::MatrixXd Sensor::getPInvDs(double Ts) const {
	 return pinv(getDs(Ts));
 }

 Sensor::Sensor(BaseSystem::BaseSystemPtr ptr, unsigned int ID) : System(ID), numOfBaseSystemStates(ptr->getNumOfStates()),
	 numOfBaseSystemDisturbances(ptr->getNumOfDisturbances()), numOfBaseSystemNoises(ptr->getNumOfNoises()) {}

 unsigned int Sensor::getNumOfBaseSystemStates() const { return numOfBaseSystemStates; }

 unsigned int Sensor::getNumOfBaseSystemNoises() const { return numOfBaseSystemNoises; }

 unsigned int Sensor::getNumOfBaseSystemDisturbances() const {
	 return numOfBaseSystemDisturbances;
 }
