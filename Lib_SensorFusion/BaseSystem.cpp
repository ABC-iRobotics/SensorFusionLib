#include "BaseSystem.h"
#include "pinv.h"
// The nonlinear parts and the dependencies are by default zeros

BaseSystem::BaseSystem(unsigned int ID)
	: System(ID) {}

Eigen::VectorXi BaseSystem::getStateUpdateNonlinXDep() const {
	return Eigen::VectorXi::Zero(getNumOfStates());
}

Eigen::VectorXi BaseSystem::getStateUpdateNonlinWDep() const {
	return Eigen::VectorXi::Zero(getNumOfDisturbances());
}

Eigen::VectorXi BaseSystem::getOutputUpdateNonlinXDep() const {
	return Eigen::VectorXi::Zero(getNumOfStates());
}

Eigen::VectorXi BaseSystem::getOutputUpdateNonlinVDep() const {
	return Eigen::VectorXi::Zero(getNumOfNoises());
}

Eigen::VectorXd BaseSystem::EvalStateUpdateNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const {
	return Eigen::VectorXd::Zero(getNumOfStates());
}

// The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions

Eigen::VectorXd BaseSystem::EvalOutputUpdateNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& noise) const {
	return Eigen::VectorXd::Zero(getNumOfOutputs());
}

Eigen::VectorXd BaseSystem::EvalStateUpdate(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const {
	return getA(Ts)*state + getB(Ts)*disturbance + EvalStateUpdateNonlinearPart(Ts, state, disturbance);
}

Eigen::VectorXd BaseSystem::EvalOutputUpdate(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& noise) const {
	return getC(Ts)*state + getD(Ts)*noise + EvalOutputUpdateNonlinearPart(Ts, state, noise);
}

Eigen::VectorXd BaseSystem::EvalNonlinearPart(TimeUpdateType type, double Ts, const Eigen::VectorXd & state, const Eigen::VectorXd & in) const {
	switch (type) {
	case STATE_UPDATE:
		return EvalStateUpdateNonlinearPart(Ts, state, in);
	case OUTPUT_UPDATE:
		return EvalOutputUpdateNonlinearPart(Ts, state, in);
	}
	throw std::runtime_error(std::string("BaseSystem::EvalNonlinearPart(): Unknown input!"));
}

Eigen::VectorXi BaseSystem::getNonlinDep(TimeUpdateType outType, VariableType inType) {
	switch (outType) {
	case STATE_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return getStateUpdateNonlinXDep();
		case VAR_EXTERNAL:
			return getStateUpdateNonlinWDep();
		}
	case OUTPUT_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return getOutputUpdateNonlinXDep();
		case VAR_EXTERNAL:
			return getOutputUpdateNonlinVDep();
		}
	}
	throw std::runtime_error(std::string("BaseSystem::getNonlinDep(): Unknown input!"));
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
	// Check pinv-s
	if (!eq(getPInvB(0.01), pinv(getB(0.01))))
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	if (!eq(getPInvD(0.01), pinv(getD(0.01))))
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	// Check dependency vectors
	Eigen::VectorXi v;
	v = getStateUpdateNonlinXDep();
	if (v.size() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	v = getStateUpdateNonlinWDep();
	if (v.size() != nw)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	v = getOutputUpdateNonlinXDep();
	if (v.size() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	v = getOutputUpdateNonlinVDep();
	if (v.size() != nv)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	// Check nonlinear parts
	Eigen::VectorXd m;
	Eigen::VectorXd x = Eigen::VectorXd::Zero(nx);
	Eigen::VectorXd i = Eigen::VectorXd::Zero(nw);
	m = EvalStateUpdateNonlinearPart(1, x, i);
	if (m.size() != nx)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
	i = Eigen::VectorXd::Zero(nv);
	m = EvalOutputUpdateNonlinearPart(1, x, i);
	if (m.size() != ny)
		throw std::runtime_error(std::string("BaseSystem::systemTest()"));
}

Eigen::MatrixXd BaseSystem::getPInvB(double Ts) const {
	return pinv(getB(Ts));
}

Eigen::MatrixXd BaseSystem::getPInvD(double Ts) const {
	return pinv(getD(Ts));
}
