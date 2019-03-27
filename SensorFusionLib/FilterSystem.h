#pragma once

#include "Sensor.h"

#include <iostream> // to delete


class FilterSystem {
private:
	unsigned int iID;

	struct BaseSystemData {
		BaseSystem::BaseSystemPtr ptr;
		StatisticValue noise;
		Eigen::VectorXd measurement;
		bool measurementUpToDate;
		BaseSystemData(BaseSystem::BaseSystemPtr ptr_);
	} baseSystemData;

	struct SensorData {
		Sensor::SensorPtr ptr;
		StatisticValue noise;
		Eigen::VectorXd measurement;
		bool measurementUpToDate;
		SensorData(Sensor::SensorPtr ptr_);
	};

	typedef std::vector<SensorData> SensorList;

	SensorList cSensors;

	struct SystemValues {
		StatisticValue state;
		StatisticValue disturbance;
		SystemValues(BaseSystem::BaseSystemPtr baseptr);
		StatisticValue Get(System::ValueType type) const;
		void Set(System::ValueType type, StatisticValue value);
	} values;

private:
	// Insert the state, variance, noise or measuredoutput of the basesystem into the vStateVector, mVarianceMatrix
	void _SetBaseSystemValue(StatisticValue value, System::ValueType type);

	// Insert the state, variance, noise or measuredoutput of a sensor into the vStateVector, mVarianceMatrix
	void _SetSensorValue(unsigned int ID, StatisticValue value, System::ValueType type);

public:
	FilterSystem(BaseSystem::BaseSystemPtr baseSystem);

	~FilterSystem();

	void AddSensor(Sensor::SensorPtr sensor); // options!

	Eigen::VectorXd GetStateVector() const;

	Eigen::MatrixXd GetVarianceMatrix() const;

	Eigen::VectorXd GetBaseSystemStateVector() const;

	Eigen::VectorXd GetSensorStateVector(int i) const;

	unsigned int GetNumOfSensors() const { return cSensors.size(); }

	Eigen::VectorXd ComputeUpdate(Eigen::VectorXd state, Eigen::VectorXd dist, double Ts) const;

	StatisticValue ComputeUpdate(double Ts, const StatisticValue& state, const StatisticValue& disturbance) const;

	StatisticValue ComputeOutput(double Ts, const StatisticValue& state, Eigen::MatrixXd& Syx, Eigen::VectorXd& y_measured) const;

	void Step(double dT) { // update, collect measurement, correction via Kalman-filtering
		StatisticValue x_pred = ComputeUpdate(dT, values.state, values.disturbance);
		Eigen::MatrixXd Syxpred;
		Eigen::VectorXd y_meas;
		StatisticValue y_pred = ComputeOutput(dT, x_pred, Syxpred, y_meas);

		Eigen::MatrixXd K = Syxpred.transpose() * y_pred.variance.inverse();

		Eigen::MatrixXd Sxnew = x_pred.variance - K * Syxpred;
		values.state = StatisticValue(x_pred.vector + K * (y_meas - y_pred.vector),
			(Sxnew + Sxnew.transpose()) / 2.);
	}

	// how to structure kalman filtering / wrwaukf ?
};

