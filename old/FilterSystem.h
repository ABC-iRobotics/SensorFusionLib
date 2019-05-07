#pragma once

#include "Sensor.h"
#include "CallbackHandler.h"
#include "StatisticValue.h"
#include <iostream> // to delete
#include <map>

#include "FunctionMerge.h"

enum FilterValueType {
	PREDICTED_STATE, PREDICTED_OUTPUT,
	USED_DISTURBANCE, USED_NOISE,
	FILTERED_STATE, FILTERED_OUTPUT,
	MEASURED_OUTPUT, DT,
	GROUND_TRUTH_STATE, T
};

typedef std::map<FilterValueType, StatisticValue> FilterData;

enum FilterEvents { STEP, MEASUREMENT };

class FilterSystem : public CallbackHandler<const FilterData&, FilterEvents> {
	
public:
	enum MeasurementStatus { OBSOLETHE, UPTODATE, CONSTANT };

	struct SystemData {
		System::SystemPtr ptr;
		StatisticValue noise;
		StatisticValue disturbance;
		Eigen::VectorXd measurement;
		MeasurementStatus measStatus;

		SystemData(System::SystemPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_=Eigen::VectorXd(), MeasurementStatus measStatus_=OBSOLETHE);
		BaseSystem::BaseSystemPtr getBaseSystemPtr() const;
		Sensor::SensorPtr getSensorPtr() const;
		StatisticValue getProperty(SystemValueType type) const;
		void setProperty(StatisticValue value, SystemValueType type);
	};

private:
	typedef std::vector<SystemData> SystemList;

	SystemList systemList;

	StatisticValue state;

	int _GetIndexForID(unsigned int ID) const;

	StatisticValue _GetSystemPropertyByID(unsigned int ID, SystemValueType type) const;
	StatisticValue _GetSystemProperty(unsigned int index, SystemValueType type) const;

	void _SetSystemProperty(unsigned int index, StatisticValue value, SystemValueType type);
	void _SetBaseSystemProperty(StatisticValue value, SystemValueType type);
	void _SetSystemPropertyByID(unsigned int ID, StatisticValue value, SystemValueType type);
	void _SetSensorPropertyByID(unsigned int ID, StatisticValue value, SystemValueType type);

	StatisticValue _PartitionateState(StatisticValue in, unsigned int systemindex) const {
		int k = 0;
		for (unsigned int i = 0; i + 1 < systemindex; i++)
			k += systemList[i].ptr->getNumOf(STATE);
		int dk = systemList[systemindex].ptr->getNumOf(STATE);
		return in.GetPart(k, dk);
	}

	std::vector<Eigen::VectorXd> _PartitionateStateVector(Eigen::VectorXd stateVector) const {
		std::vector<Eigen::VectorXd> out = std::vector<Eigen::VectorXd>();
		int k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++) {
			int dk = systemList[i].ptr->getNumOf(STATE);
			out.push_back(stateVector.segment(k, dk));
			k += dk;
		}
		return out;
	}

	std::vector<Eigen::VectorXd> _PartitionateOutputVector(Eigen::VectorXd outputVector, std::vector<bool> activeSystems) const {
		std::vector<Eigen::VectorXd> out = std::vector<Eigen::VectorXd>();
		int k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++)
			if (activeSystems[i]) {
				int dk = systemList[i].ptr->getNumOf(OUTPUT);
				out.push_back(outputVector.segment(k, dk));
				k += dk;
			}
			else
				out.push_back(Eigen::VectorXd(0));
		return out;
	}

	/*
	Eigen::VectorXd _Partitionate(SystemValueType type, Eigen::VectorXd in) const {

	}*/
	
public:
	FilterSystem(SystemData baseSystemData, StatisticValue baseSystemState);

	~FilterSystem();

	void AddSensor(SystemData sensorData, StatisticValue sensorState); // options!

	StatisticValue GetDisturbance() const;

	StatisticValue GetState() const { return state; }
	
	Eigen::VectorXd GetMeasuredOutput() const {
		int k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++)
			if (systemList[i].measStatus != OBSOLETHE)
				k += systemList[i].ptr->getNumOfOutputs();
		Eigen::VectorXd out = Eigen::VectorXd(k);
		k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++)
			if (systemList[i].measStatus != OBSOLETHE) {
				int dk = systemList[i].ptr->getNumOfOutputs();
				out.segment(k, dk) = systemList[i].measurement;
				k += dk;
			}
		return out;
	}

	StatisticValue GetNoise() const {
		int k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++)
			if (i == 0 || systemList[i].measStatus != OBSOLETHE)
				k += systemList[i].ptr->getNumOfNoises();
		StatisticValue out(k);
		k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++)
			if (i == 0 || systemList[i].measStatus != OBSOLETHE) {
				out.Insert(k, systemList[i].noise);
				k += systemList[i].ptr->getNumOfNoises();
			}
		return out;
	}

	// Outputdynamics:
	/* y = [ y_bs' y_s1' ... y_sn' ]' y_i = [] if there is not uptodate measurement
	   x = [ x_bs' x_s1' ... y_sn' ]'
	   v = [ v_bs' v_s1' ... y_sn' ]' v_si = [] if there is not updtodate measurement
	   y = C*x + f(x,v) + D*v;
	*/
	const FunctionMerge GetOutputDynamics(double Ts) const {
		// 
		FunctionMerge merger(systemList[0].getBaseSystemPtr()->getOutputMapping(Ts,
			systemList[0].measStatus == OBSOLETHE));
		for (size_t i = 1; i < systemList.size(); i++)
			merger.AddFunction(systemList[i].getSensorPtr()->getOutputMapping(Ts,
				systemList[i].measStatus == OBSOLETHE));
		return merger;
	}

	Eigen::Index GetNumOfSensors() const { return systemList.size()-1; }

	// Compute new state from exactly known state and disturbance
	Eigen::VectorXd ComputeUpdate(Eigen::VectorXd state, Eigen::VectorXd dist, double Ts) const;

	// Compute new statistical state from statistical state and disturbance
	StatisticValue ComputeUpdate(double Ts, const StatisticValue& state, const StatisticValue& disturbance) const;

	// Compute statistical output from statistical state and disturbance
	StatisticValue ComputeOutput(double Ts, const StatisticValue& state, const StatisticValue& noise_, Eigen::MatrixXd& Syx) const;

	void Step(double dT);

	// how to structure kalman filtering / wrwaukf ?

	std::ostream& print(std::ostream& stream) const;
};

std::ostream& operator<< (std::ostream& stream, const FilterSystem& filter);