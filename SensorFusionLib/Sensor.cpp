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

 // The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions

 Eigen::VectorXd Sensor::EvalUpdate(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemDisturbance, Eigen::VectorXd sensorState, Eigen::VectorXd sensorDisturbance) const {
	 return getA0(Ts)*baseSystemState + getAi(Ts)*sensorState +
		 getB0(Ts)*baseSystemDisturbance + getBi(Ts)*sensorDisturbance +
		 UpdateNonlinearPart(Ts, baseSystemState, baseSystemDisturbance, sensorState, sensorDisturbance);
 }

 Eigen::VectorXd Sensor::EvalOutput(double Ts, Eigen::VectorXd baseSystemState, Eigen::VectorXd baseSystemNoise,
	 Eigen::VectorXd sensorState, Eigen::VectorXd sensorNoise) const {
	 return getC0(Ts)*baseSystemState + getCi(Ts)*sensorState +
		 getD0(Ts)*baseSystemNoise + getDi(Ts)*sensorNoise +
		 OutputNonlinearPart(Ts, baseSystemState, baseSystemNoise, sensorState, sensorNoise);
 }

 Eigen::VectorXd Sensor::genNonlinearPart(UpdateType type, double Ts, const Eigen::VectorXd & baseSystemState,
	 const Eigen::VectorXd & baseSystemIn, const Eigen::VectorXd & sensorState, const Eigen::VectorXd & sensorIn) const {
	 switch (type) {
	 case TIMEUPDATE:
		 return UpdateNonlinearPart(Ts, baseSystemState, baseSystemIn, sensorState, sensorIn);
	 case MEASUREMENTUPDATE:
		 return OutputNonlinearPart(Ts, baseSystemState, baseSystemIn, sensorState, sensorIn);
	 }
	 throw std::runtime_error(std::string("Sensor::genNonlinearPart(): Unknown input!"));
 }

 Eigen::VectorXi Sensor::genNonlinearBaseSystemDependency(UpdateType outType, InputType inType) const {
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
	 throw std::runtime_error(std::string("Sensor::genNonlinearBaseSystemDependency(): Unknown input!"));
 }

 Eigen::VectorXi Sensor::genNonlinearSensorDependency(UpdateType outType, InputType inType) const {
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
	 throw std::runtime_error(std::string("Sensor::genNonlinearSensorDependency(): Unknown input!"));
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
	 M = getAi(1);
	 if (M.cols() != nx || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getBi(1);
	 if (M.cols() != nw || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getCi(1);
	 if (M.cols() != nx || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getDi(1);
	 if (M.cols() != nv || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 //
	 M = getA0(1);
	 if (M.cols() != nx0 || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getB0(1);
	 if (M.cols() != nw0 || M.rows() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getC0(1);
	 if (M.cols() != nx0 || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 M = getD0(1);
	 if (M.cols() != nv0 || M.rows() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 // Check dependency vectors
	 Eigen::VectorXi v;
	 v = getUpdateNonlinearX0Dependencies();
	 if (v.size() != nx0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getUpdateNonlinearW0Dependencies();
	 if (v.size() != nw0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputNonlinearX0Dependencies();
	 if (v.size() != nx0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputNonlinearV0Dependencies();
	 if (v.size() != nv0)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 //
	 v = getUpdateNonlinearXiDependencies();
	 if (v.size() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getUpdateNonlinearWiDependencies();
	 if (v.size() != nw)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputNonlinearXiDependencies();
	 if (v.size() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 v = getOutputNonlinearViDependencies();
	 if (v.size() != nv)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 // Check nonlinear parts
	 Eigen::VectorXd m;
	 Eigen::VectorXd x = Eigen::VectorXd::Zero(nx);
	 Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx0);
	 Eigen::VectorXd i = Eigen::VectorXd::Zero(nw);
	 Eigen::VectorXd i0 = Eigen::VectorXd::Zero(nw0);
	 m = UpdateNonlinearPart(1, x0, i0, x, i);
	 if (m.size() != nx)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
	 i0 = Eigen::VectorXd::Zero(nv0);
	 i = Eigen::VectorXd::Zero(nv);
	 m = OutputNonlinearPart(1, x0, i0, x, i);
	 if (m.size() != ny)
		 throw std::runtime_error(std::string("Sensor::systemTest()"));
 }

 unsigned int Sensor::getNumOfBaseSystemStates() const { return numOfBaseSystemStates; }

 unsigned int Sensor::getNumOfBaseSystemNoises() const { return numOfBaseSystemNoises; }

 unsigned int Sensor::getNumOfBaseSystemDisturbances() const {
	 return numOfBaseSystemDisturbances;
 }
