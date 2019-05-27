#include "BaseSystem.h"

// The nonlinear parts and the dependencies are by default zeros

Eigen::VectorXi BaseSystem::getUpdateNonlinearXDependencies() const {
	return Eigen::VectorXi::Zero(getNumOfStates());
}

Eigen::VectorXi BaseSystem::getUpdateNonlinearWDependencies() const {
	return Eigen::VectorXi::Zero(getNumOfDisturbances());
}

Eigen::VectorXi BaseSystem::getOutputNonlinearXDependencies() const {
	return Eigen::VectorXi::Zero(getNumOfStates());
}

Eigen::VectorXi BaseSystem::getOutputNonlinearVDependencies() const {
	return Eigen::VectorXi::Zero(getNumOfNoises());
}

Eigen::VectorXd BaseSystem::UpdateNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const {
	return Eigen::VectorXd::Zero(getNumOfStates());
}


// The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions

Eigen::VectorXd BaseSystem::OutputNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& noise) const {
	return Eigen::VectorXd::Zero(getNumOfOutputs());
}

Eigen::VectorXd BaseSystem::EvalUpdate(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const {
	return getA(Ts)*state + getB(Ts)*disturbance + UpdateNonlinearPart(Ts, state, disturbance);
}

Eigen::VectorXd BaseSystem::EvalOutput(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& noise) const {
	return getC(Ts)*state + getD(Ts)*noise + OutputNonlinearPart(Ts, state, noise);
}

Eigen::VectorXd BaseSystem::genNonlinearPart(UpdateType type, double Ts, const Eigen::VectorXd & state, const Eigen::VectorXd & in) const {
	switch (type) {
	case TIMEUPDATE:
		return UpdateNonlinearPart(Ts, state, in);
	case MEASUREMENTUPDATE:
		return OutputNonlinearPart(Ts, state, in);
	}
	throw std::runtime_error(std::string("BaseSystem::genNonlinearPart(): Unknown input!"));
}

Eigen::VectorXi BaseSystem::genNonlinearDependency(UpdateType outType, InputType inType) {
	switch (outType) {
	case TIMEUPDATE:
		switch (inType) {
		case STATE:
			return getUpdateNonlinearXDependencies();
		case INPUT:
			return getUpdateNonlinearWDependencies();
		}
	case MEASUREMENTUPDATE:
		switch (inType) {
		case STATE:
			return getOutputNonlinearXDependencies();
		case INPUT:
			return getOutputNonlinearVDependencies();
		}
	}
	throw std::runtime_error(std::string("BaseSystem::genNonlinearDependency(): Unknown input!"));
}

void BaseSystem::systemTest() const {
	_systemTest();
	int nx = getNumOfStates();
	int ny = getNumOfOutputs();
	int nv = getNumOfNoises();
	int nw = getNumOfDisturbances();
	// Check A,B,C,D matrices
	Eigen::MatrixXd M;
	M = getA(1);
	if (M.cols() != nx || M.rows() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	M = getB(1);
	if (M.cols() != nw || M.rows() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	M = getC(1);
	if (M.cols() != nx || M.rows() != ny)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	M = getD(1);
	if (M.cols() != nv || M.rows() != ny)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	// Check dependency vectors
	Eigen::VectorXi v;
	v = getUpdateNonlinearXDependencies();
	if (v.size() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	v = getUpdateNonlinearWDependencies();
	if (v.size() != nw)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	v = getOutputNonlinearXDependencies();
	if (v.size() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	v = getOutputNonlinearVDependencies();
	if (v.size() != nv)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	// Check nonlinear parts
	Eigen::VectorXd m;
	Eigen::VectorXd x = Eigen::VectorXd::Zero(nx);
	Eigen::VectorXd i = Eigen::VectorXd::Zero(nw);
	m = UpdateNonlinearPart(1, x, i);
	if (m.size() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	i = Eigen::VectorXd::Zero(nv);
	m = OutputNonlinearPart(1, x, i);
	if (m.size() != ny)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
}
