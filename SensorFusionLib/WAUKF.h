#include "SystemManager.h"
#include "Eigen/Dense"
#include <map>
#include "pinv.h"

const unsigned int MAX_WINDOW_SIZE = 200;

template<class Type>
class MAWindow { // Type will be Eigen::VectorXd or Eigen::MatrixXd
	Type data[MAX_WINDOW_SIZE];
	Type out;
	bool upToDate;
	bool initialized;
	unsigned int lastWritten;
	unsigned int windowSize;
public:
	MAWindow(unsigned int windowSize_, const Type& initValue);
	MAWindow(unsigned int windowSize_ = 100); // constructor without initialization
	const Type& Value();
	void AddValue(const Type& value);
};

class WAUKF : public SystemManager {
private:
	typedef std::map<unsigned int, MAWindow<Eigen::VectorXd>> mapOfVectorWindows;
	typedef std::map<unsigned int, MAWindow<Eigen::MatrixXd>> mapOfMatrixWindows;

	mapOfVectorWindows noiseValueWindows;
	mapOfVectorWindows disturbanceValueWindows;
	mapOfMatrixWindows noiseVarianceWindows;
	mapOfMatrixWindows disturbanceVarianceWindows;

	// fölösleges....
	const mapOfVectorWindows& _getValueWindows(SystemValueType signal) const;
	const mapOfMatrixWindows& _getVarianceWindows(SystemValueType signal) const;
	bool _isEstimated(unsigned int systemID, SystemValueType signal, ValueType type) const;

public:
	void SetDisturbanceValueWindowing(System::SystemPtr ptr, unsigned int windowSize);

	void SetNoiseValueWindowing(System::SystemPtr ptr, unsigned int windowSize);

	void SetDisturbanceVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize);

	void SetNoiseVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize);
		
	WAUKF(const BaseSystemData& data, const StatisticValue& state_);

	typedef std::shared_ptr<WAUKF> WAUKFPtr;

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

	void Step(double dT) { // update, collect measurement, correction via Kalman-filtering
		// Prediction
		Eigen::MatrixXd sg1, sg2;
		StatisticValue x_pred0, y_pred0;
		StatisticValue x_pred = EvalWithV0(EVAL_STATEUPDATE, dT, (*this)(STATE),
			(*this)(SystemValueType::DISTURBANCE), sg1, sg2, false, x_pred0);
		Eigen::MatrixXd Syxpred;
		Eigen::VectorXd y_meas = (*this)(OUTPUT).vector;
		StatisticValue y_pred = EvalWithV0(EVAL_OUTPUT, dT, x_pred,
			(*this)(SystemValueType::NOISE), Syxpred, sg2, false, y_pred0);

		StepClock(dT);
		PredictionDone(x_pred, y_pred);
		// Kalman-filtering
		Eigen::MatrixXd K = Syxpred.transpose() * y_pred.variance.inverse();
		Eigen::MatrixXd Sxnew = x_pred.variance - K * Syxpred;
		StatisticValue newstate = StatisticValue(x_pred.vector + K * (y_meas - y_pred.vector),
			(Sxnew + Sxnew.transpose()) / 2.);

		FilteringDone(newstate);
		State() = newstate;

		// Statistics estimation
		Partitioner p = getPartitioner();
		Eigen::VectorXd epsilon = y_meas - y_pred.vector;
		// DISTURBANCE
		{
			Eigen::MatrixXd pinvBbs = BaseSystem().getBaseSystemPtr()->getPInvB(dT);
			{ //Disturbance value estimation
				Eigen::VectorXd value = newstate.vector - x_pred0.vector;
				for (auto it = disturbanceValueWindows.begin(); it != disturbanceValueWindows.end(); it++) {
					int index = _GetIndex(it->first);
					Eigen::VectorXd v = pinvBbs * p.PartValue(SystemValueType::STATE, value, -1);// partx_[0];
					if (index != -1) { //basesystem
						auto sys = Sensor(index);
						Eigen::MatrixXd Bi0 = sys.getMatrixBaseSystem(dT, EVAL_STATEUPDATE, VAR_EXTERNAL, true);
						Eigen::MatrixXd pinvBi1 = sys.getSensorPtr()->getPInvBi(dT);
						v = pinvBi1 *(p.PartValue(SystemValueType::STATE, value, index) - Bi0 * v);
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
					Eigen::MatrixXd v = pinvBbs * p.PartVariance(SystemValueType::STATE, value, -1, -1) * pinvBbs.transpose();
					if (index != -1) {
						auto sys = Sensor(index);
						Eigen::MatrixXd Bi0 = sys.getMatrixBaseSystem(dT, EVAL_STATEUPDATE, VAR_EXTERNAL, true);
						Eigen::MatrixXd v2 = Bi0 * pinvBbs * p.PartVariance(SystemValueType::STATE, value, -1, index);
						Eigen::MatrixXd pinvBi1 = sys.getSensorPtr()->getPInvBi(dT);
						v = pinvBi1 * (Bi0*v*Bi0.transpose() - v2 - v2.transpose() +
							p.PartVariance(SystemValueType::STATE, value, index, index))*pinvBi1.transpose();
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
			Eigen::MatrixXd pinvDbs = BaseSystem().getBaseSystemPtr()->getPInvD(dT);
			{ //Noise value estimation
				Eigen::VectorXd value = y_meas - y_pred0.vector;
				auto party_ = partitionate(OUTPUT, y_meas - y_pred0.vector);
				for (auto it = noiseValueWindows.begin(); it != noiseValueWindows.end(); it++) {
					int index = _GetIndex(it->first);
					if (available(index)) {
						Eigen::VectorXd v = pinvDbs * p.PartValue(SystemValueType::OUTPUT, value, -1);// partx_[0];
						if (index != -1) { //basesystem
							auto sys = Sensor(index);
							Eigen::MatrixXd Di0 = sys.getMatrixBaseSystem(dT, EVAL_OUTPUT, VAR_EXTERNAL, true);
							Eigen::MatrixXd pinvDi1 = sys.getSensorPtr()->getPInvDi(dT);
							v = pinvDi1 *(p.PartValue(SystemValueType::OUTPUT, value, index) - Di0 * v);
						}
						it->second.AddValue(v); // todo: save results
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
					if (available(index)) {
						Eigen::MatrixXd v = pinvDbs * p.PartVariance(OUTPUT, value, -1, -1) * pinvDbs.transpose();
						if (index != -1) { //basesystem
							auto sys = Sensor(index);
							Eigen::MatrixXd Di0 = sys.getMatrixBaseSystem(dT, EVAL_OUTPUT, VAR_EXTERNAL, true);
							Eigen::MatrixXd v2 = Di0 * pinvDbs * p.PartVariance(OUTPUT, value, index, -1);
							Eigen::MatrixXd pinvDi1 = sys.getSensorPtr()->getPInvDi(dT);
							v = pinvDi1 * (Di0*v*Di0.transpose() - v2 - v2.transpose() +
								p.PartVariance(OUTPUT, value, index, index))*pinvDi1.transpose();
						}
						it->second.AddValue(v); // todo: save results
						if (index != -1)
							Sensor(index).setVariance(it->second.Value(), NOISE);
						else
							BaseSystem().setVariance(it->second.Value(), NOISE);
					}
				}
			}
		}
		// Reset measurement
		resetMeasurement();
	}


	StatisticValue EvalWithV0(EvalType outType, double Ts, const StatisticValue& state_,
		const StatisticValue& in, Eigen::MatrixXd & S_out_x, Eigen::MatrixXd& S_out_in,
		bool forcedOutput, StatisticValue& v0) const;

	protected:
		void _setProperty(int systemID, SystemCallData call) override {


			//TODO: ha ablakozott, akkor nem kell beállítani...

			SystemManager::_setProperty(systemID, call);
		}
};



template<class Type>
inline MAWindow<Type>::MAWindow(unsigned int windowSize_, const Type & initValue) :
	windowSize(windowSize_), initialized(true), lastWritten(0) {
	if (windowSize > MAX_WINDOW_SIZE)
		throw std::runtime_error(std::string("MAWindow::MAWindow Wrong window size to be applied!"));
	out = initValue;
	upToDate = true;
	initialized = true;
	for (unsigned int n = 0; n < MAX_WINDOW_SIZE; n++)
		data[n] = initValue;
}

template<class Type>
inline MAWindow<Type>::MAWindow(unsigned int windowSize_) :
	windowSize(windowSize_), initialized(false), lastWritten(0), upToDate(false) {}

template<class Type>
inline const Type & MAWindow<Type>::Value() {
	if (!initialized)
		throw std::runtime_error(std::string("MAWindow::Value Getter cannot be applied before initialization!"));
	if (!upToDate) {
		out = out * 0;
		int starter = lastWritten;
		for (int n = 0; n < (int)windowSize; n++) {
			if (starter - n < 0)
				starter += MAX_WINDOW_SIZE;
			out += data[starter - n];
		}
		out /= windowSize;
		upToDate = true;
	}
	return out;
}

template<class Type>
inline void MAWindow<Type>::AddValue(const Type & value) {
	if (initialized) {
		if (windowSize > 0) {
			lastWritten++;
			if (lastWritten >= MAX_WINDOW_SIZE)
				lastWritten = 0;
			data[lastWritten] = value;
			upToDate = false;
		}
		else {
			out = value;
			upToDate = true;
		}
	}
	else {
		out = value;
		upToDate = true;
		initialized = true;
		for (unsigned int n = 0; n < MAX_WINDOW_SIZE; n++)
			data[n] = value;
	}
}
