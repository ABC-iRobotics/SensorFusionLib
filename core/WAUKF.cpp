#include "WAUKF.h"
#include "PartialCholevski.h"

using namespace SF;

const WAUKF::mapOfVectorWindows & WAUKF::_getValueWindows(DataType signal) const {
	switch (signal) {
	case DISTURBANCE:
		return disturbanceValueWindows;
	case NOISE:
		return noiseValueWindows;
	default:
		throw std::runtime_error(std::string("WAUKF::getVariance Unknown option!"));
	}
}

const WAUKF::mapOfMatrixWindows & WAUKF::_getVarianceWindows(DataType signal) const {
	switch (signal) {
	case DISTURBANCE:
		return disturbanceVarianceWindows;
	case NOISE:
		return noiseVarianceWindows;
	default:
		throw std::runtime_error(std::string("WAUKF::_getVarianceWindows Unknown option!"));
	}
}

bool WAUKF::_isEstimated(unsigned int systemID, DataType type, ValueType valueorvariance) const {
	if (type == STATE || type == OUTPUT)
		return false;
	switch (valueorvariance) {
	case VALUE: {
		const mapOfVectorWindows& temp = _getValueWindows(type);
		return temp.find(systemID) != temp.end();
	}
	case VARIANCE: {
		const mapOfMatrixWindows& temp = _getVarianceWindows(type);
		return temp.find(systemID) != temp.end();
	}
	default:
		throw std::runtime_error(std::string("WAUKF::_isEstimated Unknown option!"));
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

WAUKF::WAUKF(const BaseSystemData & data, const StatisticValue & state_, const Time& t0) : SystemManager(data, state_),
	lastStepTime(t0),
	noiseValueWindows(mapOfVectorWindows()), disturbanceValueWindows(mapOfVectorWindows()),
	noiseVarianceWindows(mapOfMatrixWindows()), disturbanceVarianceWindows(mapOfMatrixWindows()) {}

StatisticValue WAUKF::_evalWithV0(TimeUpdateType outType, double Ts,
	const StatisticValue & state_, const StatisticValue & in, Eigen::MatrixXd & S_out_x,
	Eigen::MatrixXd & S_out_in, bool forcedOutput, StatisticValue & v0) const {
	DataType inType = System::getInputValueType(outType, VAR_EXTERNAL);
	Eigen::Index nX = num(STATE, forcedOutput);
	Eigen::Index nIn = num(inType, forcedOutput);
	// Get nonlinear dependencies
	Eigen::VectorXi stateDep = dep(outType, VAR_STATE, forcedOutput);
	Eigen::VectorXi inDep = dep(outType, VAR_EXTERNAL, forcedOutput);
	size_t nOut = num(System::getOutputValueType(outType), forcedOutput);
	// 
	Eigen::VectorXd z;
	Eigen::MatrixXd Sz;
	Eigen::MatrixXd Szx;
	Eigen::MatrixXd Szw;
	if (stateDep.sum() + inDep.sum() == 0) {
		z = Eigen::VectorXd::Zero(nOut);
		Sz = Eigen::MatrixXd::Zero(nOut, nOut);
		Szx = Eigen::MatrixXd::Zero(nOut, nX);
		Szw = Eigen::MatrixXd::Zero(nOut, nIn);
	}
	else {
		// Sigma values by applying partial chol
		Eigen::MatrixXd dX = PartialChol(state_.variance, stateDep);
		Eigen::MatrixXd dIn;
		if (in.isIndependent) {
			dIn = Eigen::MatrixXd::Zero(nIn, inDep.sum());
			unsigned int j = 0;
			for (unsigned int i = 0; i < nIn; i++)
				if (inDep[i] == 1) {
					dIn(i, j) = sqrt(in.variance(i, i));
					j++;
				}
		}
		else
			dIn = PartialChol(in.variance, inDep);
		Eigen::Index nNL_X = dX.cols();
		Eigen::Index nNL_In = dIn.cols();
		Eigen::Index nNL = nNL_X + nNL_In;
		// Constants for UT
		double alpha = 0.7;
		double beta = 2.;
		double kappa = 1e-8;
		double tau2 = alpha * alpha * (kappa + (double)nNL);
		double tau = sqrt(tau2);
		dX *= tau;
		dIn *= tau;

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


Eigen::MatrixXd DiagAndLimit(const Eigen::MatrixXd& in, double limit) {
	Eigen::MatrixXd out = in;
	for (unsigned int i = 0; i < out.rows(); i++)
		for (unsigned int j = 0; j < out.rows(); j++)
			if (i != j)
				out(i, j) = 0;
			else
				if (out(i, i) < limit)
					out(i, i) = limit;
	return out;
}

// TODO: further tests on the adaptive methods

void WAUKF::Step(const DTime& dT) { // update, collect measurement, correction via Kalman-filtering
									 // Prediction
	double dT_sec = duration_cast_to_sec(dT);
	Eigen::MatrixXd sg1, sg2;
	StatisticValue x_pred0, y_pred0;
	StatisticValue x_pred = _evalWithV0(STATE_UPDATE, dT_sec, (*this)(STATE),
		(*this)(DataType::DISTURBANCE), sg1, sg2, false, x_pred0);
	Eigen::MatrixXd Syxpred;
	StatisticValue y_meas = (*this)(OUTPUT);
	StatisticValue y_pred = _evalWithV0(OUTPUT_UPDATE, dT_sec, x_pred,
		(*this)(DataType::NOISE), Syxpred, sg2, false, y_pred0);

	PredictionDone(x_pred, y_pred);
	// Kalman-filtering
	Eigen::MatrixXd Syy = y_pred.variance + y_meas.variance;
	Eigen::MatrixXd K = Syxpred.transpose() * Syy.inverse();
	Eigen::MatrixXd Sxnew = x_pred.variance - K * Syxpred;
	StatisticValue newstate = StatisticValue(x_pred.vector + K * (y_meas.vector - y_pred.vector),
		(Sxnew + Sxnew.transpose()) / 2.);

	FilteringDone(newstate);
	State() = newstate;

	// Statistics estimation
	Partitioner p = getPartitioner();
	Eigen::VectorXd epsilon = y_meas.vector - y_pred.vector;
	// DISTURBANCE
	{
		Eigen::MatrixXd pinvBbs = BaseSystem().getBaseSystemPtr()->getPInvB(dT_sec);
		{ //Disturbance value estimation
			Eigen::VectorXd value = newstate.vector - x_pred0.vector;
			for (auto it = disturbanceValueWindows.begin(); it != disturbanceValueWindows.end(); it++) {
				int index = _GetIndex(it->first);
				Eigen::VectorXd v = pinvBbs * p.PartValue(DataType::STATE, value, -1);// partx_[0];
				if (index != -1) { //basesystem
					auto sys = Sensor(index);
					Eigen::MatrixXd Bi0 = sys.getMatrixBaseSystem(dT_sec, STATE_UPDATE, VAR_EXTERNAL, true);
					Eigen::MatrixXd pinvBi1 = sys.getSensorPtr()->getPInvBs(dT_sec);
					v = pinvBi1 * (p.PartValue(DataType::STATE, value, index) - Bi0 * v);
				}
				it->second.AddValue(v);
				if (index != -1)
					Sensor(index).setValue(it->second.Value(), DISTURBANCE);
				else
					BaseSystem().setValue(it->second.Value(), DISTURBANCE);
			}
		}
		{ // Disturbance variance
			Eigen::MatrixXd value = K * epsilon;
			value = newstate.variance + value * value.transpose() - x_pred0.variance;
			for (auto it = disturbanceVarianceWindows.begin(); it != disturbanceVarianceWindows.end(); it++) {
				int index = _GetIndex(it->first);
				Eigen::MatrixXd v = pinvBbs * p.PartVariance(DataType::STATE, value, -1, -1) * pinvBbs.transpose();
				if (index != -1) {
					auto sys = Sensor(index);
					Eigen::MatrixXd Bi0 = sys.getMatrixBaseSystem(dT_sec, STATE_UPDATE, VAR_EXTERNAL, true);
					Eigen::MatrixXd v2 = Bi0 * pinvBbs * p.PartVariance(DataType::STATE, value, -1, index);
					Eigen::MatrixXd pinvBi1 = sys.getSensorPtr()->getPInvBs(dT_sec);
					v = pinvBi1 * (Bi0*v*Bi0.transpose() - v2 - v2.transpose() +
						p.PartVariance(DataType::STATE, value, index, index))*pinvBi1.transpose();
				}
				it->second.AddValue(DiagAndLimit(v, 0.00001));
				Eigen::MatrixXd out = it->second.Value();
				if (index != -1)
					Sensor(index).setVariance(out, DISTURBANCE);
				else
					BaseSystem().setVariance(out, DISTURBANCE);
			}
		}
	}
	// NOISE
	{
		Eigen::MatrixXd pinvDbs = BaseSystem().getBaseSystemPtr()->getPInvD(dT_sec);
		{ //Noise value estimation
			Eigen::VectorXd value = y_meas.vector - y_pred0.vector;
			for (auto it = noiseValueWindows.begin(); it != noiseValueWindows.end(); it++) {
				int index = _GetIndex(it->first);
				if (isAvailable(index)) {
					Eigen::VectorXd v = pinvDbs * p.PartValue(DataType::OUTPUT, value, -1);// partx_[0];
					if (index != -1) { //basesystem
						auto sys = Sensor(index);
						Eigen::MatrixXd Di0 = sys.getMatrixBaseSystem(dT_sec, OUTPUT_UPDATE, VAR_EXTERNAL, true);
						Eigen::MatrixXd pinvDi1 = sys.getSensorPtr()->getPInvDs(dT_sec);
						v = pinvDi1 * (p.PartValue(DataType::OUTPUT, value, index) - Di0 * v);
					}
					it->second.AddValue(v);
					if (index != -1)
						Sensor(index).setValue(it->second.Value(), NOISE);
					else
						BaseSystem().setValue(it->second.Value(), NOISE);
				}
			}
		}
		{ // Noise variance
			Eigen::MatrixXd value = epsilon * epsilon.transpose() - y_pred0.variance;
			for (auto it = noiseVarianceWindows.begin(); it != noiseVarianceWindows.end(); it++) {
				int index = _GetIndex(it->first);
				if (isAvailable(index)) {
					Eigen::MatrixXd v = pinvDbs * p.PartVariance(OUTPUT, value, -1, -1) * pinvDbs.transpose();
					if (index != -1) { //basesystem
						auto sys = Sensor(index);
						Eigen::MatrixXd Di0 = sys.getMatrixBaseSystem(dT_sec, OUTPUT_UPDATE, VAR_EXTERNAL, true);
						Eigen::MatrixXd v2 = Di0 * pinvDbs * p.PartVariance(OUTPUT, value, -1, index); // index sorrend?
						Eigen::MatrixXd pinvDi1 = sys.getSensorPtr()->getPInvDs(dT_sec);
						v = pinvDi1 * (Di0*v*Di0.transpose() - v2 - v2.transpose() +
							p.PartVariance(OUTPUT, value, index, index))*pinvDi1.transpose();
					}
					it->second.AddValue(v);
					Eigen::MatrixXd res = DiagAndLimit(it->second.Value(), 0.00001);
					//std::cout << "S_vv_0: \n" << res << std::endl;
					if (index != -1)
						Sensor(index).setVariance(res, NOISE);
					else
						BaseSystem().setVariance(res, NOISE);
				}
			}
		}
	}
	// Reset measurement
	resetMeasurement();
}

void WAUKF::SaveDataMsg(const DataMsg& data, const Time& t) {
	auto data_ = data;
	if (_isEstimated(data.GetSourceID(), data.GetDataType(), VALUE))
		data_.ClearValue();
	if (_isEstimated(data.GetSourceID(), data.GetDataType(), VARIANCE))
		data_.ClearVariance();
	SystemManager::SaveDataMsg(data_, t);
}

void SF::WAUKF::SamplingTimeOver(const Time & currentTime) {
	Step(duration_cast(currentTime - lastStepTime));
}

void SF::WAUKF::MsgQueueEmpty(const Time & currentTime) {
	Step(duration_cast(currentTime - lastStepTime));
}