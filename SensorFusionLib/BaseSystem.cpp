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

Eigen::VectorXd BaseSystem::genNonlinearPart(UpdateType type, double Ts, const Eigen::VectorXd & state, const Eigen::VectorXd & in) const {
	switch (type) {
	case TIMEUPDATE:
		return UpdateNonlinearPart(Ts, state, in);
	case MEASUREMENTUPDATE:
		return OutputNonlinearPart(Ts, state, in);
	}
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
}
