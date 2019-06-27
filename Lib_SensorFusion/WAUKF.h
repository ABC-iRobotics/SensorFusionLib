#include "SystemManager.h"
#include "Eigen/Dense"
#include <map>

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
public:
	WAUKF(const BaseSystemData& data, const StatisticValue& state_);

	void SetDisturbanceValueWindowing(System::SystemPtr ptr, unsigned int windowSize);

	void SetNoiseValueWindowing(System::SystemPtr ptr, unsigned int windowSize);

	void SetDisturbanceVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize);

	void SetNoiseVarianceWindowing(System::SystemPtr ptr, unsigned int windowSize);

	typedef std::shared_ptr<WAUKF> WAUKFPtr;

	// TODO: perform further tests on the adaptive methods
	void Step(TimeMicroSec dT) override;

	void SetProperty(const DataMsg& data) override;

private:
	typedef std::map<unsigned int, MAWindow<Eigen::VectorXd>> mapOfVectorWindows;
	typedef std::map<unsigned int, MAWindow<Eigen::MatrixXd>> mapOfMatrixWindows;

	mapOfVectorWindows noiseValueWindows;
	mapOfVectorWindows disturbanceValueWindows;
	mapOfMatrixWindows noiseVarianceWindows;
	mapOfMatrixWindows disturbanceVarianceWindows;

	const mapOfVectorWindows& _getValueWindows(DataType signal) const;
	const mapOfMatrixWindows& _getVarianceWindows(DataType signal) const;
	bool _isEstimated(unsigned int systemID, DataType signal, ValueType type) const;

	StatisticValue _evalWithV0(TimeUpdateType outType, double Ts, const StatisticValue& state_,
		const StatisticValue& in, Eigen::MatrixXd & S_out_x, Eigen::MatrixXd& S_out_in,
		bool forcedOutput, StatisticValue& v0) const;
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
