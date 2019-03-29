#include "pch.h"
#include "FilterSystem.h"
#include "FunctionMerge.h"

void FilterSystem::_SetBaseSystemValue(StatisticValue value, SystemValueType type) {
	if (type == STATE || type == DISTURBANCE) {
		StatisticValue v = values.Get(type);
		unsigned int n = v.Length();
		unsigned int dn = baseSystemData.ptr->getNumOf(type);
		for (unsigned int i = 0; i < dn; i++) {
			v.vector(i) = value.vector(i);
			for (unsigned int j = 0; j < dn; j++)
				v.variance(i, j) = value.variance(i, j);
			for (unsigned int j = dn; j < n; j++)
				v.variance(i, j) = v.variance(j, i) = 0.;
		}
		values.Set(type, v);
		return;
	}
	// Measurement & Noise
	switch (type) {
	case NOISE:
		baseSystemData.noise = value;
		return;
	case OUTPUT:
		baseSystemData.measurement = value.vector;
		baseSystemData.measurementUpToDate = true;
		return;
	}
}

void FilterSystem::_SetSensorValue(unsigned int ID, StatisticValue value, SystemValueType type) {
	if (type == STATE || type == DISTURBANCE) {
		StatisticValue v = values.Get(type);
		unsigned int n = baseSystemData.ptr->getNumOf(type);
		for (unsigned int i = 0; i < cSensors.size(); i++) {
			unsigned int dn = cSensors[i].ptr->getNumOf(type);
			if (cSensors[i].ptr->getID() == ID) {
				for (unsigned int j = 0; j < dn; j++) {
					v.vector(n + j) = value.vector(j);
					for (unsigned int j2 = 0; j2 < dn; j2++)
						v.variance(n + j, n + j2) = value.variance(j, j2);
					for (unsigned int j2 = 0; j2 < v.Length(); j2++)
						if (j2 < n || j2 >= n + dn)
							v.variance(n + j, j2) = v.variance(j2, n + j) = 0;
				}
				values.Set(type, v);
				return;
			}
			else n += dn;
		}
		std::cout << "(FilterSystem::_SetSensorValue) Unknown sensor ID\n";
	}
	// Measurement & Noise
	if (type == NOISE || type == OUTPUT) {
		for (unsigned int i = 0; i < cSensors.size(); i++)
			if (cSensors[i].ptr->getID() == ID) {
				switch (type) {
				case NOISE:
					cSensors[i].noise = value;
					return;
				case OUTPUT:
					cSensors[i].measurement = value.vector;
					cSensors[i].measurementUpToDate = true;
					return;
				}
			}
		std::cout << "(FilterSystem::_SetSensorValue) Unknown sensor ID\n";
	}
}

FilterSystem::FilterSystem(BaseSystem::BaseSystemPtr baseSystem) :
	baseSystemData(baseSystem), cSensors(SensorList()), values(baseSystem) {
	// Set and step ID
	static unsigned int ID = 0;
	iID = ID;
	ID++;
	// set callback
	baseSystemData.ptr->AddCallback([this](StatisticValue value, SystemValueType type) {
		this->_SetBaseSystemValue(std::move(value), std::move(type)); },
		iID);
}

FilterSystem::~FilterSystem() {
	baseSystemData.ptr->DeleteCallback(iID);
	for (unsigned int i = 0; i < cSensors.size(); i++)
		cSensors[i].ptr->DeleteCallback(iID);
}

void FilterSystem::AddSensor(Sensor::SensorPtr sensor) {
	if (sensor->isCompatible(baseSystemData.ptr)) {
		//Add to the list
		cSensors.push_back(SensorData(sensor));
		// Get its ID
		unsigned int sensorID = sensor->getID();
		// Add the initial state values and variances to the state/variance matrix
		for (const auto type : { STATE, DISTURBANCE }) {
			StatisticValue v = values.Get(type);
			int n0 = v.Length();
			int dn = sensor->getNumOf(type);
			v.vector.conservativeResize(n0 + dn);
			v.variance.conservativeResize(n0 + dn, n0 + dn);
			values.Set(type, v);
			_SetSensorValue(sensorID, sensor->getInitValue(type), type);
		}
		// Set callbacks
		sensor->AddCallback([this, sensorID](StatisticValue value,
			SystemValueType type) {
			this->_SetSensorValue(sensorID, std::move(value), std::move(type)); },
			iID);
	}
	else std::cout << "Not compatible sensor tried to be added!\n";
}

Eigen::VectorXd FilterSystem::GetStateVector() const {
	return values.state.vector;
}

Eigen::MatrixXd FilterSystem::GetVarianceMatrix() const {
	return values.state.variance;
}

Eigen::VectorXd FilterSystem::GetBaseSystemStateVector() const {
	return values.state.vector.segment(0, baseSystemData.ptr->getNumOfStates());
}

Eigen::VectorXd FilterSystem::GetSensorStateVector(int i) const {
	int k = baseSystemData.ptr->getNumOfStates();
	for (int j = 0; j < i - 1; j++)
		k += cSensors[j].ptr->getNumOfStates();
	return values.state.vector.segment(k, k + cSensors[i].ptr->getNumOfStates());
}

Eigen::VectorXd FilterSystem::ComputeUpdate(Eigen::VectorXd state, Eigen::VectorXd dist, double Ts) const { //todo restructure / test
	int is0, dis;
	is0 = 0;
	dis = baseSystemData.ptr->getNumOfStates();
	Eigen::VectorXd systemstate = state.segment(is0, dis);
	int id0, did;
	id0 = 0;
	did = baseSystemData.ptr->getNumOfDisturbances();
	Eigen::VectorXd systemdist = dist.segment(id0, did);
	Eigen::VectorXd out = baseSystemData.ptr->EvalUpdate(Ts, systemstate, systemdist);
	for (unsigned int i = 0; i < GetNumOfSensors(); i++) {
		is0 += dis;
		dis = cSensors[i].ptr->getNumOfStates();
		id0 += did;
		did = cSensors[i].ptr->getNumOfDisturbances();
		out << out, cSensors[i].ptr->EvalUpdate(Ts, systemstate, systemdist,
			state.segment(is0, dis), dist.segment(id0, did));
	}
	return out;
}

StatisticValue FilterSystem::ComputeUpdate(double Ts, const StatisticValue& state, const StatisticValue& disturbance) const {
	FunctionMerge merger(baseSystemData.ptr->getUpdateMapping(Ts));
	for (unsigned int i = 0; i < cSensors.size(); i++)
		merger.AddFunction(cSensors[i].ptr->getUpdateMapping(Ts));
	Eigen::MatrixXd Syx, Syz;
	return merger.Eval(state, disturbance, Syx, Syz);
}

StatisticValue FilterSystem::ComputeOutput(double Ts, const StatisticValue& state,
	Eigen::MatrixXd& Syx, Eigen::VectorXd& y_measured) const {
	// Compute the number of outputs & noises
	unsigned int x0Size = baseSystemData.ptr->getNumOfStates();
	unsigned int v0Size = baseSystemData.ptr->getNumOfNoises();
	unsigned int nv = v0Size;
	unsigned int ny = baseSystemData.ptr->getNumOfOutputs();
	for (unsigned int i = 0; i < cSensors.size(); i++)
		if (cSensors[i].measurementUpToDate) {
			nv += cSensors[i].noise.Length();
			ny += cSensors[i].measurement.size();
		}
	// Alocate variables
	y_measured = Eigen::VectorXd(ny);
	StatisticValue v(nv);
	// Fill the allocated variables and construct the merger
	v.Insert(0, baseSystemData.noise);
	ny = baseSystemData.ptr->getNumOfOutputs();
	nv = v0Size;
	FunctionMerge merger(baseSystemData.ptr->getUpdateMapping(Ts));
	for (unsigned int i = 0; i < ny; i++)
		y_measured(i) = baseSystemData.measurement(i);
	for (unsigned int i = 0; i < cSensors.size(); i++)
		if (cSensors[i].measurementUpToDate) {
			v.Insert(nv, cSensors[i].noise);
			nv += cSensors[i].noise.Length();
			unsigned int dy = cSensors[i].measurement.size();
			for (unsigned int j = 0; j < dy; j++)
				y_measured(ny + j) = cSensors[i].measurement(j);
			ny += dy;
			merger.AddFunction(cSensors[i].ptr->getUpdateMapping(Ts));
		}
		else merger.AddFunction(Function4::Empty(x0Size, v0Size,
			cSensors[i].ptr->getNumOfStates(), 0));
	// Perform computation
	Eigen::MatrixXd Syz;
	return merger.Eval(state, v, Syx, Syz);
}

FilterSystem::BaseSystemData::BaseSystemData(BaseSystem::BaseSystemPtr ptr_) : ptr(ptr_),
noise(ptr->getInitializationNoises()), measurementUpToDate(false) {}

FilterSystem::SensorData::SensorData(Sensor::SensorPtr ptr_) : ptr(ptr_),
measurementUpToDate(false), noise(ptr_->getInitializationNoises()) {}

FilterSystem::SystemValues::SystemValues(BaseSystem::BaseSystemPtr baseptr) : state(baseptr->getInitializationStates()),
disturbance(baseptr->getInitializationDisturbances()) {}

StatisticValue FilterSystem::SystemValues::Get(SystemValueType type) const {
	switch (type) {
	case DISTURBANCE:
		return disturbance;
	case STATE:
		return state;
	default:
		return StatisticValue();
	}
}

void FilterSystem::SystemValues::Set(SystemValueType type, StatisticValue value) {
	switch (type) {
	case DISTURBANCE:
		disturbance = value;
		return;
	case STATE:
		state = value;
		return;
	}
}
