#include "pch.h"
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

Function2 BaseSystem::getUpdateMapping(double Ts) const {
	return Function2(getA(Ts), getB(Ts), getUpdateNonlinearXDependencies(), getUpdateNonlinearWDependencies(),
		[this, Ts](const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) {
		return this->UpdateNonlinearPart(Ts, std::move(state), std::move(disturbance)); });
}

Function2 BaseSystem::getOutputMapping(double Ts, bool empty) const {
	// if empty, num of outputs are zero
	if (empty) {
		int xSize = getNumOfStates();
		int vSize = getNumOfNoises();
		return Function2(Eigen::MatrixXd(0, xSize), Eigen::MatrixXd(0, vSize),
			Eigen::VectorXi::Zero(xSize), Eigen::VectorXi::Zero(vSize),
			[](const Eigen::VectorXd& state, const Eigen::VectorXd& noise) {
			return Eigen::VectorXd(0); });
	}
	else
		return Function2(getC(Ts), getD(Ts), getOutputNonlinearXDependencies(), getOutputNonlinearVDependencies(),
			[this, Ts](const Eigen::VectorXd& state, const Eigen::VectorXd& noise) {
			return this->OutputNonlinearPart(Ts, std::move(state), std::move(noise)); });
}
