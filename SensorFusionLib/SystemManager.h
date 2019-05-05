#pragma once

#include "Sensor.h"

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

Output vector: y = [ y_bs' y_s1' ... y_sn' ]' y_i = [] if there is not uptodate measurement
Noise vector: v = [ v_bs' v_s1' ... v_sn' ]' v_si = [] if there is not updtodate measurement

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
Noise vector: v = [ v_bs' v_s1' ... v_sn' ]' v_si = [] if there is not updtodate measurement

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

struct FilterCallData {
	StatisticValue value;
	System::SystemPtr ptr;
	double t;
	SystemValueType type;
	FilterCallData(StatisticValue value_, System::SystemPtr ptr_, double t_, SystemValueType type_) :
		value(value_), ptr(ptr_), t(t_), type(type_) {};
};

enum FilterCallType { PREDICTION, FILTERING, MEASUREMENT, ESTIMATION, GROUNDTRUTH };

class SystemManager : public CallbackHandler<const FilterCallData&, FilterCallType> {
public:
	enum MeasurementStatus { OBSOLETHE, UPTODATE, CONSTANT };

	class SystemData {
		MeasurementStatus measStatus;
		StatisticValue noise;
		StatisticValue disturbance;
		Eigen::VectorXd measurement;
	public:
		SystemData(StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_ = Eigen::VectorXd(), MeasurementStatus measStatus_ = OBSOLETHE);
		virtual System::SystemPtr getPtr() const = 0;
		StatisticValue operator()(SystemValueType type, bool forcedOutput = false) const; // returns th given value
		size_t num(SystemValueType type, bool forcedOutput = false) const; // return length of the given value accroding to the measStatus
		void set(StatisticValue value, SystemValueType type); // set the given value
		void resetMeasurement();
		bool available() const; // returns if is measurement available
		virtual bool isBaseSystem() const = 0;
	};

	class BaseSystemData : public SystemData {
		BaseSystem::BaseSystemPtr ptr;
	public:
		BaseSystemData(BaseSystem::BaseSystemPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_ = Eigen::VectorXd(), MeasurementStatus measStatus_ = OBSOLETHE);
		Eigen::VectorXi dep(System::UpdateType outType, System::InputType type,
			bool forcedOutput = false) const;
		Eigen::MatrixXd getMatrix(double Ts, System::UpdateType type,
			System::InputType inType, bool forcedOutput = false) const;
		BaseSystem::BaseSystemPtr getBaseSystemPtr() const;
		System::SystemPtr getPtr() const override;
		bool isBaseSystem() const override;
	};

	class SensorData : public SystemData {
		Sensor::SensorPtr ptr;
	public:
		SensorData(Sensor::SensorPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_ = Eigen::VectorXd(), MeasurementStatus measStatus_ = OBSOLETHE);
		Eigen::VectorXi depSensor(System::UpdateType outType, System::InputType type,
			bool forcedOutput = false) const;
		Eigen::VectorXi depBaseSystem(System::UpdateType outType, System::InputType type,
			bool forcedOutput = false) const;
		Eigen::MatrixXd getMatrixBaseSystem(double Ts, System::UpdateType type,
			System::InputType inType, bool forcedOutput = false) const;
		Eigen::MatrixXd getMatrixSensor(double Ts, System::UpdateType type, System::InputType inType,
			bool forcedOutput = false) const;
		Sensor::SensorPtr getSensorPtr() const;
		System::SystemPtr getPtr() const override;;
		bool isBaseSystem() const override;
	};

	// Constructor: the basesystem  must be given as an input
	SystemManager(BaseSystemData data, StatisticValue state_);

	// Add a sensor
	void AddSensor(SensorData sensorData, StatisticValue sensorState);

	size_t nSensors() const;

	const SystemData* SystemByID(unsigned int ID) const;

	size_t num(SystemValueType type, bool forcedOutput = false) const;

	Eigen::VectorXi dep(System::UpdateType outType, System::InputType inType, bool forcedOutput = false) const;

	/* Get A,B, C,D matrices according to the available sensors*/
	void getMatrices(System::UpdateType out_, double Ts, Eigen::MatrixXd& A,
		Eigen::MatrixXd& B, bool forcedOutput = false) const;

	// could be faster....
	Eigen::VectorXd EvalNonLinPart(double Ts, System::UpdateType outType,
		Eigen::VectorXd state, Eigen::VectorXd in, bool forcedOutput = false) const;

	// This could be faster by implementing EvalNonLinPart and partitionate to matrices
	StatisticValue Eval(System::UpdateType outType, double Ts, StatisticValue state_, StatisticValue in,
		Eigen::MatrixXd& S_out_x, Eigen::MatrixXd S_out_in, bool forcedOutput = false) const;

	// Get the STATE, DISTURBANCE, measured OUTPUT, NOISE vectors for the basesystem and the active sensors
	StatisticValue operator()(SystemValueType type, bool forcedOutput = false) const;

	std::ostream & print(std::ostream & stream) const;

	// Destructor
	~SystemManager();

protected:
	int _GetIndex(unsigned int ID) const; // returns -1 for the basesystem!
	BaseSystemData & BaseSystem();
	SensorData & Sensor(size_t index);
	StatisticValue& State();
	SystemData* SystemByID(unsigned int ID);
	const SensorData& Sensor(size_t index) const;

	// Partitionate back the STATE, DISTURBANCE, measured OUTPUT, NOISE vectors for the basesystem and the active sensors
	std::vector<Eigen::VectorXd> partitionate(SystemValueType type,
		Eigen::VectorXd value, bool forcedOutput = false) const;

	// Partitionate back the STATE, DISTURBANCE, measured OUTPUT, NOISE vectors for the basesystem and the active sensors
	std::vector<StatisticValue> partitionateWithStatistic(SystemValueType type,
		StatisticValue value, bool forcedOutput = false) const;

	void resetMeasurement();

	void PredictionDone(StatisticValue state, StatisticValue output, double t) const;

	void FilteringDone(StatisticValue state, double t) const;

private:
	unsigned int ID; // unique ID for identifying callbacks

	typedef std::vector<SensorData> SensorList;
	SensorList sensorList;
	BaseSystemData baseSystem;

	StatisticValue state;
};



