#include "pch.h"
#include "FilterSystem.h"
#include "FunctionMerge.h"

int FilterSystem::_GetIndexForID(unsigned int ID) const {
	for (unsigned int i = 0; i < systemList.size(); i++)
		if (systemList[i].ptr->getID() == ID)
			return i;
	assert(true);
	return -1;
}

void FilterSystem::_SetSystemPropertyByID(unsigned int ID, StatisticValue value, SystemValueType type) {
	switch (type)
	{
	case STATE:
	{
		int k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++) {
			int dk = systemList[i].ptr->getNumOfStates();
			if (systemList[i].ptr->getID() == ID) {
				state.Insert(k, value);
				return;
			}
			k += dk;
		}
		assert(true);
	}
	default:
		systemList[_GetIndexForID(ID)].setProperty(value, type);
		return;
	}
}

StatisticValue FilterSystem::_GetSystemPropertyByID(unsigned int ID, SystemValueType type) const {
	switch (type)
	{
	case STATE:
	{
		int k = 0;
		for (unsigned int i = 0; i < systemList.size(); i++) {
			int dk = systemList[i].ptr->getNumOf(type);
			if (systemList[i].ptr->getID() == ID)
				return state.GetPart(k, dk);
			k += dk;
		}
		assert(true);
	}
	default:
		return systemList[_GetIndexForID(ID)].getProperty(type);
	}
}

void FilterSystem::_SetSystemProperty(unsigned int index, StatisticValue value, SystemValueType type) {
	switch (type)
	{
	case STATE:
	{
		int k = 0;
		for (unsigned int i = 0; i+1 < index; i++)
			k += systemList[i].ptr->getNumOf(type);
		state.Insert(k, value);
		return;
	}
	default:
		systemList[index].setProperty(value, type);
		return;
	}
}

StatisticValue FilterSystem::_GetSystemProperty(unsigned int index, SystemValueType type) const {
	switch (type)
	{
	case STATE:
	{
		int k = 0;
		for (unsigned int i = 0; i+1 < index; i++)
			k += systemList[i].ptr->getNumOf(type);
		int dk = systemList[index].ptr->getNumOf(type);
		return state.GetPart(k, dk);
	}
	default:
		return systemList[index].getProperty(type);
	}
}

// Insert the state, variance, noise or measuredoutput of the basesystem into the vStateVector, mVarianceMatrix

void FilterSystem::_SetBaseSystemProperty(StatisticValue value, SystemValueType type) {
	_SetSystemProperty(0, value, type);
}

// Insert the state, variance, noise or measuredoutput of a sensor into the vStateVector, mVarianceMatrix

void FilterSystem::_SetSensorPropertyByID(unsigned int ID, StatisticValue value, SystemValueType type) {
	_SetSystemPropertyByID(ID, value, type);
}

FilterSystem::FilterSystem(SystemData data, StatisticValue state_) :
	systemList(SystemList()), state(state_) {
	systemList.push_back(data);
	// set callback
	data.ptr->AddCallback([this](Eigen::VectorXd value, EmptyClass) {
		this->_SetBaseSystemProperty(StatisticValue(value), SystemValueType::OUTPUT); },
		getID());
}

FilterSystem::~FilterSystem() {
	for (unsigned int i = 0; i < systemList.size(); i++)
		systemList[i].ptr->DeleteCallback(getID());
}

void FilterSystem::AddSensor(SystemData data, StatisticValue state_) {
	if (data.getSensorPtr()->isCompatible(systemList[0].getBaseSystemPtr())) {
		//Add to the list
		systemList.push_back(data);
		// Add the initial state values and variances to the state/variance matrix
		state.Add(state_);
		// Set callbacks
		unsigned int sensorID = data.ptr->getID();
		data.ptr->AddCallback([this, sensorID](Eigen::VectorXd value,
			EmptyClass) {
			this->_SetSensorPropertyByID(sensorID, std::move(value), SystemValueType::OUTPUT); },
			getID());
	}
	else std::cout << "Not compatible sensor tried to be added!\n";
}

StatisticValue FilterSystem::GetDisturbance() const {
	int k = 0;
	for (unsigned int i = 0; i < systemList.size(); i++)
		k += systemList[i].ptr->getNumOfDisturbances();
	StatisticValue out(k);
	k = 0;
	for (unsigned int i = 0; i < systemList.size(); i++) {
		out.Insert(k, systemList[i].disturbance);
		k += systemList[i].ptr->getNumOfDisturbances();
	}
	return out;
}

/*
Eigen::VectorXd FilterSystem::GetBaseSystemStateVector() const {
	return state.vector.segment(0, systemList[0].ptr->getNumOfStates());
}

Eigen::VectorXd FilterSystem::GetSensorStateVector(int i) const {
	int k = 0;
	for (int j = 0; j < i; j++)
		k += systemList[j].ptr->getNumOfStates();
	return state.vector.segment(k, k + systemList[i].ptr->getNumOfStates());
}*/


Eigen::VectorXd FilterSystem::ComputeUpdate(Eigen::VectorXd state, Eigen::VectorXd dist, double Ts) const { //todo restructure / test
	int is0, dis;
	is0 = 0;
	dis = systemList[0].ptr->getNumOfStates();
	Eigen::VectorXd systemstate = state.segment(is0, dis);
	int id0, did;
	id0 = 0;
	did = systemList[0].ptr->getNumOfDisturbances();
	Eigen::VectorXd systemdist = dist.segment(id0, did);
	Eigen::VectorXd out = systemList[0].getBaseSystemPtr()->EvalUpdate(Ts, systemstate, systemdist);
	for (unsigned int i = 1; i <= GetNumOfSensors(); i++) {
		is0 += dis;
		dis = systemList[i].ptr->getNumOfStates();
		id0 += did;
		did = systemList[i].ptr->getNumOfDisturbances();
		out << out, systemList[i].getSensorPtr()->EvalUpdate(Ts, systemstate, systemdist,
			state.segment(is0, dis), dist.segment(id0, did));
	}
	return out;
}

StatisticValue FilterSystem::ComputeUpdate(double Ts, const StatisticValue& state, const StatisticValue& disturbance) const {
	FunctionMerge merger(systemList[0].getBaseSystemPtr()->getUpdateMapping(Ts));
	for (unsigned int i = 1; i < systemList.size(); i++)
		merger.AddFunction(systemList[i].getSensorPtr()->getUpdateMapping(Ts));
	Eigen::MatrixXd Syx, Syz;
	return merger.Eval(state, disturbance, Syx, Syz);
}

StatisticValue FilterSystem::ComputeOutput(double Ts, const StatisticValue& state_, const StatisticValue& noise_,
	Eigen::MatrixXd& Syx) const {
	// Construct the merger
	FunctionMerge merger = GetOutputDynamics(Ts);
	// Perform computation
	Eigen::MatrixXd Syz;
	return merger.Eval(state_, noise_, Syx, Syz);
}

void FilterSystem::Step(double dT) { // update, collect measurement, correction via Kalman-filtering
	StatisticValue x_pred = ComputeUpdate(dT, state, GetDisturbance());
	Eigen::MatrixXd Syxpred;
	Eigen::VectorXd y_meas = GetMeasuredOutput();
	StatisticValue y_pred = ComputeOutput(dT, x_pred, GetNoise(), Syxpred);
	//std::cout << y_pred;
	Eigen::MatrixXd K = Syxpred.transpose() * y_pred.variance.inverse();
	//::cout << K;
	Eigen::MatrixXd Sxnew = x_pred.variance - K * Syxpred;
	state = StatisticValue(x_pred.vector + K * (y_meas - y_pred.vector),
		(Sxnew + Sxnew.transpose()) / 2.);

	// downgrade measStatus
	for (size_t i = 1; i < systemList.size(); i++)
		if (systemList[i].measStatus == UPTODATE)
			systemList[i].measStatus = OBSOLETHE;

	FilterData data = FilterData();
	data[PREDICTED_STATE] = x_pred;
	data[PREDICTED_OUTPUT] = y_pred;
	data[FILTERED_STATE] = state;
	data[MEASURED_OUTPUT] = y_meas;
	data[DT] = StatisticValue(Eigen::VectorXd::Ones(1)*dT);
	Call(data, STEP);
}

std::ostream & FilterSystem::print(std::ostream & stream) const {
	std::vector<Eigen::VectorXd> states = _PartitionateStateVector(state.vector);
	StatisticValue output = ComputeUpdate(0.001, state, GetDisturbance());
	std::vector<bool> active = std::vector<bool>();
	for (unsigned int i = 0; i < systemList.size(); i++)
		active.push_back(true);
	std::vector<Eigen::VectorXd> outputs = _PartitionateOutputVector(output.vector, active);
	unsigned int sys_n = systemList.size();
	for (unsigned int sys_i = 0; sys_i < sys_n; sys_i++) {
		if (sys_i == 0)
			stream << "Basesystem: ";
		else
			stream << "Sensor " << sys_i << ": ";
		stream << systemList[sys_i].ptr->getName() << std::endl;
		///// STATES
		stream << " States:";
		std::vector<std::string>& temp = systemList[sys_i].ptr->getStateNames();
		for (unsigned int i = 0; i < states[sys_i].size(); i++)
			stream << " (" << temp[i] << "): " << states[sys_i](i);
		stream << std::endl;
		///// DISTURBANCES
		stream << " Disturbances:";
		temp = systemList[sys_i].ptr->getDisturbanceNames();
		for (unsigned int i = 0; i < systemList[sys_i].disturbance.Length(); i++)
			stream << " (" << temp[i] << "): " << systemList[sys_i].disturbance.vector(i);
		stream << std::endl;
		///// NOISES
		stream << " Noises:";
		temp = systemList[sys_i].ptr->getNoiseNames();
		for (unsigned int i = 0; i < systemList[sys_i].noise.Length(); i++)
			stream << " (" << temp[i] << "): " << systemList[sys_i].noise.vector(i);
		stream << std::endl;
		///// OUTPUTS
		stream << " Outputs:\n";
		stream << "   computed:";
		temp = systemList[sys_i].ptr->getOutputNames();
		for (unsigned int i = 0; i < outputs[sys_i].size(); i++)
			stream << " (" << temp[i] << "): " << outputs[sys_i](i);
		stream << std::endl;
		if (systemList[sys_i].measStatus != OBSOLETHE) {
			stream << "   measured:";
			for (unsigned int i = 0; i < outputs[sys_i].size(); i++)
				stream << " (" << temp[i] << "): " << systemList[sys_i].measurement(i);
		}
		stream << std::endl << std::endl;
	}

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
	return stream;
}

FilterSystem::SystemData::SystemData(System::SystemPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
	Eigen::VectorXd measurement_, MeasurementStatus measStatus_) :
	ptr(ptr_), noise(noise_), measurement(measurement_), disturbance(disturbance_),
	measStatus(measStatus_) {}

BaseSystem::BaseSystemPtr FilterSystem::SystemData::getBaseSystemPtr() const {
	return std::static_pointer_cast<BaseSystem>(ptr);
}

Sensor::SensorPtr FilterSystem::SystemData::getSensorPtr() const {
	return std::static_pointer_cast<Sensor>(ptr);
}

StatisticValue FilterSystem::SystemData::getProperty(SystemValueType type) const {
	switch (type)
	{
	case NOISE:
		return noise;
	case DISTURBANCE:
		return disturbance;
	case OUTPUT:
		return measurement;
	default:
		assert(true);
		return -1;
	}
}

void FilterSystem::SystemData::setProperty(StatisticValue value, SystemValueType type) {
	switch (type)
	{
	case NOISE:
		noise = value;
		return;
	case DISTURBANCE:
		disturbance = value;
		return;
	case OUTPUT:
		measurement = value.vector;
		if (measStatus == OBSOLETHE)
			measStatus = UPTODATE;
		if (!value.variance.isZero())
			assert(true);
		return;
	default:
		assert(true);
	}
}

std::ostream& operator<< (std::ostream& stream, const FilterSystem& filter) {
	filter.print(stream);
	return stream;
}