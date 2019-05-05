#pragma once

#include "Sensor.h"

/* Only to store/describe the system and provide an interface to their interface -> it does not perform filtering
*/
class SystemManager
{
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
		StatisticValue operator()(SystemValueType type) const; // returns th given value
		size_t num(SystemValueType type) const; // return length of the given value accroding to the measStatus
		size_t num0(SystemValueType type) const; // return length of the given value
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
		Eigen::VectorXi dep(System::UpdateType outType, System::InputType type) const;
		Eigen::MatrixXd getMatrix(double Ts, System::UpdateType type, System::InputType inType) const;
		BaseSystem::BaseSystemPtr getBaseSystemPtr() const;
		System::SystemPtr getPtr() const override;
		bool isBaseSystem() const override;
	};

	class SensorData : public SystemData {
		Sensor::SensorPtr ptr;
	public:
		SensorData(Sensor::SensorPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_ = Eigen::VectorXd(), MeasurementStatus measStatus_ = OBSOLETHE);
		Eigen::VectorXi depSensor(System::UpdateType outType, System::InputType type) const;
		Eigen::VectorXi depBaseSystem(System::UpdateType outType, System::InputType type) const;
		Eigen::MatrixXd getMatrixBaseSystem(double Ts, System::UpdateType type, System::InputType inType) const;
		Eigen::MatrixXd getMatrixSensor(double Ts, System::UpdateType type, System::InputType inType) const;
		Sensor::SensorPtr getSensorPtr() const;
		System::SystemPtr getPtr() const override;;
		bool isBaseSystem() const override;
	};

private:
	unsigned int ID; // unique ID for identifying callbacks

	typedef std::vector<SensorData> SensorList;
	SensorList sensorList;
	BaseSystemData baseSystem;

	StatisticValue state;
//protected:
public:

	int _GetIndex(unsigned int ID) const;

	size_t nSensors() const;

	size_t num(SystemValueType type) const;

	SensorData Sensor(size_t index) const;
	SensorData & Sensor(size_t index);

	const SystemData* SystemByID(unsigned int ID) const;
	SystemData* SystemByID(unsigned int ID);

	// Get the whole STATE, DISTURBANCE values as a StatisticValue instance
	// OR
	// get the measured OUTPUT for the available (=not obsolethe) systems (sensors & basesystem)
	// OR
	// get the noises for the basesystem and the active sensors
	StatisticValue operator()(SystemValueType type) const;

	/* Signals:
	   x = [ x_bs' x_s1' ... x_sn' ]'
	   w = [ w_bs' w_s1' ... w_sn' ]'
	   y = [ y_bs' y_s1' ... y_sn' ]' y_i = [] if there is not uptodate measurement
	   v = [ v_bs' v_s1' ... v_sn' ]' v_si = [] if there is not updtodate measurement
	   Update model:
	   x = A*x + f(x,w) + B*w
	   Output model:
	   y = C*x + g(x,v) + D*v;
	*/

	// Partitionate back the STATE, DISTURBANCE vectors
	// OR
	// the measured OUTPUT for the available (=not obsolethe) systems (sensors & basesystem)
	// OR
	// the noises for the basesystem and the active sensors
	std::vector<Eigen::VectorXd> partitionate(SystemValueType type, Eigen::VectorXd value) const;

	/* Get A,B, C,D matrices according to the available sensors*/
	void getMatrices(System::UpdateType out_, double Ts, Eigen::MatrixXd& A, Eigen::MatrixXd& B) const;

	// could be faster....
	Eigen::VectorXd EvalNonLinPart(double Ts, System::UpdateType outType, Eigen::VectorXd state, Eigen::VectorXd in) const;

	StatisticValue Eval(System::UpdateType outType, double Ts, StatisticValue state_, StatisticValue in,
		Eigen::MatrixXd& S_out_x, Eigen::MatrixXd S_out_in) const;

public:

	void saveMeasurement(unsigned int ID, Eigen::VectorXd value);

	SystemManager(BaseSystemData data, StatisticValue state_);

	~SystemManager();

	void AddSensor(SensorData sensorData, StatisticValue sensorState);

	std::ostream & print(std::ostream & stream) const {
		std::vector<Eigen::VectorXd> states = partitionate(STATE, state.vector);
		Eigen::MatrixXd S1, S2;
		StatisticValue output = Eval(System::MEASUREMENTUPDATE, 0.001, state, (*this)(NOISE), S1, S2);
		/*std::vector<bool> active = std::vector<bool>();
		for (unsigned int i = 0; i < systemList.size(); i++)
			active.push_back(true);*/
		std::vector<Eigen::VectorXd> outputs = partitionate(OUTPUT, output.vector);
		

		

		auto printSystem = [](std::ostream& stream, const SystemData* sys,
			const Eigen::VectorXd& state, const Eigen::VectorXd& output) {
			auto printRowVector = [](std::ostream& stream, const Eigen::VectorXd& v, const std::vector<std::string>& names) {
				for (unsigned int i = 0; i < v.size(); i++)
					stream << " (" << names[i] << "): " << v(i);
				stream << std::endl;
			};
			stream << sys->getPtr()->getName() << std::endl;
			///// STATES
			stream << " States:";
			printRowVector(stream, state, sys->getPtr()->getStateNames());
			///// DISTURBANCES
			stream << " Disturbances:";
			printRowVector(stream, (*sys)(DISTURBANCE).vector, sys->getPtr()->getDisturbanceNames());
			///// NOISES
			stream << " Noises:";
			printRowVector(stream, (*sys)(NOISE).vector, sys->getPtr()->getNoiseNames());
			///// OUTPUTS
			stream << " Outputs:\n";
			stream << "   computed:";
			printRowVector(stream, output, sys->getPtr()->getOutputNames());
			if (sys->available()) {
				stream << "   measured:";
				printRowVector(stream, (*sys)(OUTPUT).vector, sys->getPtr()->getOutputNames());
			}
			stream << std::endl;
		};

		stream << "Basesystem: ";
		printSystem(stream, &baseSystem, states[0], outputs[0]);
		for (unsigned int sensor_i = 0; sensor_i < nSensors(); sensor_i++) {
			stream << "Sensor " << sensor_i << ": ";
			printSystem(stream, &sensorList[sensor_i], states[sensor_i+1], outputs[sensor_i+1]);
		}

		/*
		// STATE variances
		stream << "Variance matrix of the state:\n";
		unsigned int a = 0, b = 0;
		for (unsigned int i = 0; i < sys_n; i++) {
			for (unsigned int di = 0; di < systemList[i].ptr->getNumOfStates(); di++) {
				for (unsigned int j = 0; j < sys_n; j++) {
					for (unsigned int dj = 0; dj < systemList[j].ptr->getNumOfStates(); dj++) {
						stream << state.variance(a, b) << " ";
						b++;
					}
					if (j + 1 < sys_n)
						stream << "| ";
				}
				stream << std::endl;
				b = 0;
				a++;
			}
			if (i + 1 < sys_n)
				for (unsigned int k = 0; k < 40; k++)
					stream << "-";
			stream << std::endl;
		}
		*/
		return stream;
	}
};



