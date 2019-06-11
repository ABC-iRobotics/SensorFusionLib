#include "WAUKF.h"
#include "PartialCholevski.h"

WAUKF::mapOfVectorWindows& WAUKF::_getValueWindows(SystemValueType signal) {
	switch (signal) {
	case DISTURBANCE:
		return disturbanceValueWindows;
	case NOISE:
		return noiseValueWindows;
	default:
		throw std::runtime_error(std::string("WAUKF::_getValueWindows Unknown option!"));
	}
}

WAUKF::mapOfMatrixWindows & WAUKF::_getVarianceWindows(SystemValueType signal) {
	switch (signal) {
	case DISTURBANCE:
		return disturbanceVarianceWindows;
	case NOISE:
		return noiseVarianceWindows;
	default:
		throw std::runtime_error(std::string("WAUKF::_getVarianceWindows Unknown option!"));
	}
}

const WAUKF::mapOfVectorWindows & WAUKF::_getValueWindows(SystemValueType signal) const {
	switch (signal) {
	case DISTURBANCE:
		return disturbanceValueWindows;
	case NOISE:
		return noiseValueWindows;
	default:
		throw std::runtime_error(std::string("WAUKF::getVariance Unknown option!"));
	}
}

const WAUKF::mapOfMatrixWindows & WAUKF::_getVarianceWindows(SystemValueType signal) const {
	switch (signal) {
	case DISTURBANCE:
		return disturbanceVarianceWindows;
	case NOISE:
		return noiseVarianceWindows;
	default:
		throw std::runtime_error(std::string("WAUKF::_getVarianceWindows Unknown option!"));
	}
}

bool WAUKF::_isEstimated(unsigned int systemID, SystemValueType signal, ValueType type) const {
	switch (type) {
	case VALUE: {
		const mapOfVectorWindows& temp = _getValueWindows(signal);
		return temp.find(systemID) != temp.end();
	}
	case VARIANCE: {
		const mapOfMatrixWindows& temp = _getVarianceWindows(signal);
		return temp.find(systemID) != temp.end();
	}
	default:
		throw std::runtime_error(std::string("WAUKF::_isEstimated Unknown option!"));
	}
}

MAWindow<Eigen::VectorXd>& WAUKF::_getVectorWindow(unsigned int systemID, SystemValueType signal, bool & found) {
	mapOfVectorWindows& temp = _getValueWindows(signal);
	auto it = temp.find(systemID);
	found = it != temp.end();
	if (found) return it->second;
	else {
		static MAWindow<Eigen::VectorXd> w(0);
		return w;
	}
}

MAWindow<Eigen::MatrixXd>& WAUKF::_getMatrixWindow(unsigned int systemID, SystemValueType signal, bool & found) {
	mapOfMatrixWindows& temp = _getVarianceWindows(signal);
	auto it = temp.find(systemID);
	found = it != temp.end();
	if (found) return it->second;
	else {
		static MAWindow<Eigen::MatrixXd> w(0);
		return w;
	}
}

void WAUKF::SetDisturbanceValueWindowing(System::SystemPtr ptr, unsigned int windowSize) {
	disturbanceValueWindows.insert(std::pair<unsigned int,
		MAWindow<Eigen::VectorXd>>(ptr->getID(), MAWindow<Eigen::VectorXd>(windowSize,
			SystemByID(ptr->getID())->getValue(DISTURBANCE))));
}

void WAUKF::SetNoiseValueWindowing(System::SystemPtr ptr, unsigned int windowSize) {
	noiseValueWindows.insert(std::pair<unsigned int,
		MAWindow<Eigen::VectorXd>>(ptr->getID(), MAWindow<Eigen::VectorXd>(windowSize,
			SystemByID(ptr->getID())->getValue(NOISE))));
}

void WAUKF::SetDisturbanceVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize) {
	disturbanceVarianceWindows.insert(std::pair<unsigned int,
		MAWindow<Eigen::MatrixXd>>(ptr->getID(), MAWindow<Eigen::MatrixXd>(windowSize,
			SystemByID(ptr->getID())->getVariance(DISTURBANCE))));
}

void WAUKF::SetNoiseVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize) {
	noiseVarianceWindows.insert(std::pair<unsigned int,
		MAWindow<Eigen::MatrixXd>>(ptr->getID(), MAWindow<Eigen::MatrixXd>(windowSize,
			SystemByID(ptr->getID())->getVariance(NOISE))));
}

WAUKF::WAUKF(const BaseSystemData & data, const StatisticValue & state_) : SystemManager(data, state_),
	noiseValueWindows(mapOfVectorWindows()), disturbanceValueWindows(mapOfVectorWindows()),
	noiseVarianceWindows(mapOfMatrixWindows()), disturbanceVarianceWindows(mapOfMatrixWindows()) {}

StatisticValue WAUKF::EvalWithV0(EvalType outType, double Ts,
	const StatisticValue & state_, const StatisticValue & in, Eigen::MatrixXd & S_out_x,
	Eigen::MatrixXd & S_out_in, bool forcedOutput, StatisticValue & v0) const {
	SystemValueType inType = System::getInputValueType(outType, VAR_EXTERNAL);
	Eigen::Index nX = num(STATE, forcedOutput);
	Eigen::Index nIn = num(inType, forcedOutput);
	// Get nonlinear dependencies
	Eigen::VectorXi stateDep = dep(outType, VAR_STATE, forcedOutput);
	Eigen::VectorXi inDep = dep(outType, VAR_EXTERNAL, forcedOutput);
	Eigen::Index nNL_X = stateDep.sum();
	Eigen::Index nNL_In = inDep.sum();
	Eigen::Index nNL = nNL_X + nNL_In;
	size_t nOut = num(System::getOutputValueType(outType), forcedOutput);
	// 
	Eigen::VectorXd z;
	Eigen::MatrixXd Sz;
	Eigen::MatrixXd Szx;
	Eigen::MatrixXd Szw;
	if (nNL == 0) {
		z = Eigen::VectorXd::Zero(nOut);
		Sz = Eigen::MatrixXd::Zero(nOut, nOut);
		Szx = Eigen::MatrixXd::Zero(nOut, nX);
		Szw = Eigen::MatrixXd::Zero(nOut, nIn);
	}
	else {
		// Constants for UT
		double alpha = 0.3;
		double beta = 2.;
		double kappa = 1e-8;
		double tau2 = alpha * alpha * (kappa + (double)nNL);
		double tau = sqrt(tau2);
		// Sigma values by applying partial chol
		Eigen::MatrixXd dX = tau * PartialChol(state_.variance, stateDep);
		Eigen::MatrixXd dIn;
		if (in.isIndependent) {
			dIn = Eigen::MatrixXd::Zero(nIn, nNL_In);
			unsigned int j = 0;
			for (unsigned int i = 0; i < nIn; i++)
				if (inDep[i] == 1) {
					dIn(i, j) = tau * sqrt(in.variance(i, i));
					j++;
				}
		}
		else
			dIn = tau * PartialChol(in.variance, inDep);
		// Mean value
		Eigen::VectorXd z0 = EvalNonLinPart(Ts, outType, state_.vector, in.vector, forcedOutput);
		std::vector<Eigen::VectorXd> z_x = std::vector<Eigen::VectorXd>();

		for (Eigen::Index i = 0; i < nNL_X; i++) {
			z_x.push_back(EvalNonLinPart(Ts, outType, state_.vector + dX.col(i), in.vector, forcedOutput));
			z_x.push_back(EvalNonLinPart(Ts, outType, state_.vector - dX.col(i), in.vector, forcedOutput));
		}
		std::vector<Eigen::VectorXd> z_w = std::vector<Eigen::VectorXd>();
		for (Eigen::Index i = 0; i < nNL_In; i++) {
			z_w.push_back(EvalNonLinPart(Ts, outType, state_.vector, in.vector + dIn.col(i), forcedOutput));
			z_w.push_back(EvalNonLinPart(Ts, outType, state_.vector, in.vector - dIn.col(i), forcedOutput));
		}
		z = (1. - (double)nNL / tau2) * z0;
		for (int i = 0; i < 2 * nNL_X; i++)
			z += z_x[i] / 2. / tau2;
		for (int i = 0; i < 2 * nNL_In; i++)
			z += z_w[i] / 2. / tau2;

		Sz = ((tau2 - (double)nNL) / tau2 + 1. + beta - alpha * alpha) * (z0 - z) * (z0 - z).transpose();
		Szx = Eigen::MatrixXd::Zero(nOut, nX);
		Szw = Eigen::MatrixXd::Zero(nOut, nIn);
		Eigen::VectorXd temp;
		for (unsigned int i = 0; i < nNL_X; i++) {
			temp = z_x[i] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			temp = z_x[i + nNL_X] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			Szx += (z_x[2 * i] - z_x[2 * i + 1])*dX.col(i).transpose() / 2. / tau2;
		}
		for (int i = 0; i < nNL_In; i++) {
			temp = z_w[i] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			temp = z_w[i + nNL_In] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			Szw += (z_w[2 * i] - z_w[2 * i + 1])*dIn.col(i).transpose() / 2. / tau2;
		}
	}
	// Get coefficient matrices
	Eigen::MatrixXd A, B;
	getMatrices(outType, Ts, A, B, forcedOutput);

	Eigen::VectorXd y = A * state_.vector + z;

	S_out_x = A * state_.variance + Szx;
	S_out_in = B * in.variance + Szw;
	Eigen::MatrixXd Sy = S_out_x * A.transpose() + Szw * B.transpose() +
		Sz + A * Szx.transpose() + B * Szw.transpose();

	v0 = StatisticValue(y, Sy);
	Sy += B * in.variance * B.transpose();
	y += B * in.vector;
	return StatisticValue(y, Sy);
}
