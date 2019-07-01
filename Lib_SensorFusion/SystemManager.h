#pragma once

#include "Sensor.h"
#include "DataMsg.h"

/* SystemManager: to manage the basesystem and randomly available sensor results
----------------------------------------------------------
---Time update model:

System statevector: x = [ x_bs' x_s1' ... x_sn' ]'
Disturbance vector: w = [ w_bs' w_s1' ... w_sn' ]'

x(k) = A*x(k-1) + B*w(k) + f(x(k-1), w(k))
where
f() depends only on the entries of x, w values given by "dep" vectors
A = | Abs  0   0 ... 0   | B = | Bbs  0   0  ... 0  |
    | Abs1 As1 0 ... 0   |     | Bbs1 Bs1 0  ... 0  |
	| Abs2 0   As2 . 0   |     | Bbs2 0   Bs2... 0  |
	| .    .   . ... .   |     | .    .   .  ... .  |
	| Absn 0   0 ... Asn |     | Bbsn .   .  ... Bsn|

----------------------------------------------------------
---The measurementupdate model if every sensor output are available:

Output vector: y = [ y_bs' y_s1' ... y_sn' ]'
Noise vector: v = [ v_bs' v_s1' ... v_sn' ]'

y(k) = C*x(k) + D*v(k) + g(x(k),v(k));
where
f() depends only on the entries of x, w values given by "dep" vectors
C = | Cbs  0   0 ... 0   | D = | Dbs  0   0  ... 0  |
	| Cbs1 Cs1 0 ... 0   |     | Dbs1 Ds1 0  ... 0  |
	| Cbs2 0   Cs2 . 0   |     | Dbs2 0   Ds2... 0  |
	| .    .   . ... .   |     | .    .   .  ... .  |
	| Cbsn 0   0 ... Csn |     | Dbsn .   .  ... Dsn|

----------------------------------------------------------
---If there are sensors without uptodate output:

Output vector: y = [ y_bs' y_s1' ... y_sn' ]' y_i = [] if there is not uptodate measurement
Noise vector: v = [ v_bs' v_s1' ... v_sn' ]' v_si = [] if there is not uptodate measurement

y(k) = C*x(k) + D*v(k) + g(x(k),v(k));
where
f() depends only on the entries of x, w values given by "dep" vectors
C = | Cbs  0   0 ... 0   | D = | Dbs  0   0  ... 0  |
	| Cbs1 Cs1 0 ... 0   |     | Dbs1 Ds1 0  ... 0  |
	| Cbs2 0   Cs2 . 0   |     | Dbs2 0   Ds2... 0  |
	| .    .   . ... .   |     | .    .   .  ... .  |
	| Cbsn 0   0 ... Csn |     | Dbsn .   .  ... Dsn|
where
the row of C, D block matrices that corresponds to basesystem/sensor with obsolethe measurement has row-size 0,
the column of the D block matrix that corresponds to a sensor with obsolethe measurement has column-size 0,

----------------------------------------------------------
---Functionality:

- Build the system:
Constructor: Initiate with a basesystem and its state
AddSensor: Add a sensor and its state

- Usage:
System::SaveMeasurement: By calling  on the basesystem/sensors, it will saved in the system
operator(): to get the merged STATE, OUTPUT, DISTURBANCE, NOISE values according to the actual models
Eval(TIMEUPDATE/MEASUREMENTUPDATE, Ts, state, input): from the statistic values computes the output and its variance
print(std::ostream): prints the model details and actual properties 

details:
num(): get number of STATE, OUTPUT, DISTURBANCE, NOISE values according to the actual models
dep(TIMEUPDATE/MEASUREMENTUPDATE): get number of STATE, OUTPUT, DISTURBANCE, NOISE values according to the actual models
getMatrices(TIMEUPDATE/MEASUREMENTUPDATE, Ts, ...): get the actual matrices according to the actual models
EvalNonLinPart(TIMEUPDATE/MEASUREMENTUPDATE, Ts, ...): computes the results of f(), g() functions
*/

#include "DataMsg.h"

class SystemManager {
public:
	enum MeasurementStatus { OBSOLETHE, UPTODATE, CONSTANT };

	class SystemData {
		MeasurementStatus measStatus;
		StatisticValue noise, disturbance, measurement;
	public:
		SystemData(const StatisticValue& noise_, const StatisticValue& disturbance_, unsigned int outputSize);
		SystemData(const StatisticValue& noise_, const StatisticValue& disturbance_,
			const StatisticValue& measurement_, MeasurementStatus measStatus_);
		virtual System::SystemPtr getPtr() const = 0;
		StatisticValue operator()(DataType type, bool forcedOutput = false) const; // returns th given value
		size_t num(DataType type, bool forcedOutput = false) const; // return length of the given value accroding to the measStatus
		void setValue(const Eigen::VectorXd& value, DataType type); // set the given value
		void setVariance(const Eigen::MatrixXd& value, DataType type); // set the given value
		Eigen::VectorXd getValue(DataType type) const;
		Eigen::MatrixXd getVariance(DataType type) const;
		void resetMeasurement();
		bool available() const; // returns if is measurement available
		virtual bool isBaseSystem() const = 0;
	};

	class BaseSystemData : public SystemData {
		BaseSystem::BaseSystemPtr ptr;
	public:
		BaseSystemData(BaseSystem::BaseSystemPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_);
		BaseSystemData(BaseSystem::BaseSystemPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_, const StatisticValue& measurement_ ,
			MeasurementStatus measStatus_);
		Eigen::VectorXi dep(TimeUpdateType outType, VariableType type,
			bool forcedOutput = false) const;
		Eigen::MatrixXd getMatrix(double Ts, TimeUpdateType type,
			VariableType inType, bool forcedOutput = false) const;
		BaseSystem::BaseSystemPtr getBaseSystemPtr() const;
		System::SystemPtr getPtr() const override;
		bool isBaseSystem() const override;
	};

	class SensorData : public SystemData {
		Sensor::SensorPtr ptr;
	public:
		SensorData(Sensor::SensorPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_);
		SensorData(Sensor::SensorPtr ptr_, const StatisticValue& noise_,
			const StatisticValue& disturbance_, const StatisticValue& measurement_,
			MeasurementStatus measStatus_);
		Eigen::VectorXi depSensor(TimeUpdateType outType, VariableType type,
			bool forcedOutput = false) const;
		Eigen::VectorXi depBaseSystem(TimeUpdateType outType, VariableType type,
			bool forcedOutput = false) const;
		Eigen::MatrixXd getMatrixBaseSystem(double Ts, TimeUpdateType type,
			VariableType inType, bool forcedOutput = false) const;
		Eigen::MatrixXd getMatrixSensor(double Ts, TimeUpdateType type, VariableType inType,
			bool forcedOutput = false) const;
		Sensor::SensorPtr getSensorPtr() const;
		System::SystemPtr getPtr() const override;;
		bool isBaseSystem() const override;
	};

	/// Initialization:

	// Constructor: the basesystem  must be given as an input
	SystemManager(const BaseSystemData& data, const StatisticValue& state_);

	// Destructor
	~SystemManager();

	typedef std::shared_ptr<SystemManager> SystemManagerPtr;

	/// Usage:

	// Add a sensor
	void AddSensor(const SensorData& sensorData, const StatisticValue& sensorState);

	// The filtering function to be defined
	virtual void Step(TimeMicroSec Ts) = 0;

	// The function to inject data (meas. results, noise, disturbance value and/or variances)
	virtual void SetProperty(const DataMsg& data);

	/// To set callback for filtering results

	typedef std::function<void(const DataMsg& data)> Callback;

	void SetCallback(Callback callback_);

	void ClearCallback();

	/// Getters:

	size_t nSensors() const;

	const SystemData* SystemByID(unsigned int ID) const;

	size_t num(DataType type, bool forcedOutput = false) const;

	Eigen::VectorXi dep(TimeUpdateType outType, VariableType inType, bool forcedOutput = false) const;

	/* Get A,B, C,D matrices according to the available sensors*/
	void getMatrices(TimeUpdateType out_, double Ts, Eigen::MatrixXd& A,
		Eigen::MatrixXd& B, bool forcedOutput = false) const;

	// Get the STATE, DISTURBANCE, measured OUTPUT, NOISE vectors for the basesystem and the active sensors
	StatisticValue operator()(DataType type, bool forcedOutput = false) const;

	std::ostream & print(std::ostream & stream) const;

	/// Eval model

	// could be faster....
	Eigen::VectorXd EvalNonLinPart(double Ts, TimeUpdateType outType,
		const Eigen::VectorXd& state, const Eigen::VectorXd& in, bool forcedOutput = false) const;

	// This could be faster by implementing EvalNonLinPart and partitionate to matrices
	StatisticValue Eval(TimeUpdateType outType, double Ts, const StatisticValue& state_, const StatisticValue& in,
		Eigen::MatrixXd& S_out_x, Eigen::MatrixXd& S_out_in, bool forcedOutput = false) const;

protected:
	// Partitionate back the STATE, DISTURBANCE vectors
	// OR
	// the measured OUTPUT for the available (=not obsolethe) systems (sensors & basesystem)
	// OR
	// the noises for the basesystem and the active sensors
	// Partitionate back the STATE, DISTURBANCE, measured OUTPUT, NOISE vectors for the basesystem and the active sensors
	struct Partitioner {
		std::vector<size_t> nx;
		std::vector<size_t> nw;
		std::vector<size_t> ny;
		std::vector<size_t> nv;
		Partitioner(size_t N);
		const std::vector<size_t>& n(DataType type) const;
		Eigen::VectorBlock<Eigen::VectorXd> PartValue(DataType type,
			Eigen::VectorXd& value, int index) const;
		Eigen::VectorXd PartValue(DataType type,
			const Eigen::VectorXd& value, int index) const;
		Eigen::Block<Eigen::MatrixXd> PartVariance(DataType type,
			Eigen::MatrixXd& value, int index1, int index2) const;
		Eigen::MatrixXd PartVariance(DataType type,
			const Eigen::MatrixXd& value, int index1, int index2) const;
		Eigen::Block<Eigen::MatrixXd> PartVariance(DataType type1, DataType type2,
			Eigen::MatrixXd& value, int index1, int index2) const;
		StatisticValue PartStatisticValue(DataType type, const StatisticValue& value, int index) const;
	};
	Partitioner getPartitioner(bool forcedOutput = false) const;

	// Get index for an ID, it returns -1 for the basesystem!
	int _GetIndex(unsigned int ID) const;

	// Returns the basesystemdata ref.
	BaseSystemData & BaseSystem();
	const BaseSystemData & BaseSystem() const;
	SensorData & Sensor(size_t index);
	StatisticValue& State();
	SystemData* SystemByID(unsigned int ID);
	const SensorData& Sensor(size_t index) const;
	bool isAvailable(int index) const;

	// Set 'UPTODATE' meas. statuses to 'OBSOLETHE'
	void resetMeasurement();

	// Call it with predicition results to forward them to the set callback
	void PredictionDone(const StatisticValue& state, const StatisticValue& output) const;

	// Call it with filtering results to forward them to the set callback
	void FilteringDone(const StatisticValue& state) const;

	// To step the inner clock in the overridden Step() function
	void StepClock(TimeMicroSec dt);

	// Call the callback if it is set
	void Call(const DataMsg& data) const;

private:
	BaseSystemData baseSystem;
	std::vector<SensorData> sensorList;
	StatisticValue state;
	TimeMicroSec time;
	bool hasCallback;
	Callback callback;
};