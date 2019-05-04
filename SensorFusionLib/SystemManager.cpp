#include "SystemManager.h"

SystemManager::SystemData::SystemData(System::SystemPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
	Eigen::VectorXd measurement_, MeasurementStatus measStatus_) :
	ptr(ptr_), noise(noise_), measurement(measurement_), disturbance(disturbance_),
	measStatus(measStatus_), isBaseSystem(dynamic_cast<BaseSystem*>(ptr_.get())) {}

BaseSystem::BaseSystemPtr SystemManager::SystemData::getBaseSystemPtr() const {
	return std::static_pointer_cast<BaseSystem>(ptr);
}

Sensor::SensorPtr SystemManager::SystemData::getSensorPtr() const {
	return std::static_pointer_cast<Sensor>(ptr);
}

System::SystemPtr SystemManager::SystemData::getPtr() const { return ptr; }

StatisticValue SystemManager::SystemData::operator()(SystemValueType type) const {
	switch (type)
	{
	case SystemValueType::NOISE:
		return noise;
	case SystemValueType::DISTURBANCE:
		return disturbance;
	case SystemValueType::OUTPUT:
		return measurement;
	default:
		throw std::runtime_error(std::string("SystemManager::SystemData::operator() Only NOISE, DISTURBANCE, OUTPUT is available!"));
		return -1;
	}
}

size_t SystemManager::SystemData::num(SystemValueType type) const
{
	if (available() || type==STATE || type==DISTURBANCE || (isBaseSystem && type==NOISE))
		return ptr->getNumOf(type);
	else return 0;
}

size_t SystemManager::SystemData::num0(SystemValueType type) const { return ptr->getNumOf(type); }

// return length of the given value

Eigen::VectorXi SystemManager::SystemData::depBaseSystem(System::UpdateType outType, System::InputType type) const {
	if (outType == System::TIMEUPDATE || available()) {
		if (isBaseSystem) return getBaseSystemPtr()->genNonlinearDependency(outType, type);
		else return getSensorPtr()->genNonlinearBaseSystemDependency(outType, type);
	}
	else return Eigen::VectorXi::Zero(num0(System::getInputValueType(outType, type)));
}

Eigen::VectorXi SystemManager::SystemData::depSensor(System::UpdateType outType, System::InputType type) const {
	if (outType == System::TIMEUPDATE || available())
		return getSensorPtr()->genNonlinearSensorDependency(outType, type);
	else return Eigen::VectorXi::Zero(num0(System::getInputValueType(outType, type)));
}

void SystemManager::SystemData::set(StatisticValue value, SystemValueType type) {
	switch (type)
	{
	case SystemValueType::NOISE:
		noise = value;
		return;
	case SystemValueType::DISTURBANCE:
		disturbance = value;
		return;
	case SystemValueType::OUTPUT:
		measurement = value.vector;
		if (measStatus == OBSOLETHE)
			measStatus = UPTODATE;
		if (!value.variance.isZero())
			throw std::runtime_error(std::string("Component not available."));
		return;
	default:
		throw std::runtime_error(std::string("Component not available."));
	}
}

void SystemManager::SystemData::resetMeasurement() {
	if (measStatus == UPTODATE)
		measStatus = OBSOLETHE;
}

bool SystemManager::SystemData::available() const { return measStatus != OBSOLETHE; }

// returns if is measurement available

Eigen::MatrixXd SystemManager::SystemData::getMatrixBaseSystem(double Ts, System::UpdateType type, System::InputType inType) const {
	if (type == System::MEASUREMENTUPDATE && !available()) {
		switch (inType) {
		case System::STATE:
			return Eigen::MatrixXd(0, getSensorPtr()->getNumOfBaseSystemStates());
		case System::INPUT:
			return Eigen::MatrixXd(0, getSensorPtr()->getNumOfBaseSystemNoises());
		}
	}
	switch (type) {
	case System::TIMEUPDATE:
		switch (inType) {
		case System::STATE:
			if (isBaseSystem) return getBaseSystemPtr()->getA(Ts);
			else return getSensorPtr()->getA0(Ts);
		case System::INPUT:
			if (isBaseSystem) return getBaseSystemPtr()->getB(Ts);
			else return getSensorPtr()->getB0(Ts);
		}
	case System::MEASUREMENTUPDATE:
		switch (inType) {
		case System::STATE:
			if (isBaseSystem) return getBaseSystemPtr()->getC(Ts);
			else return getSensorPtr()->getC0(Ts);
		case System::INPUT:
			if (isBaseSystem) return getBaseSystemPtr()->getD(Ts);
			else return getSensorPtr()->getD0(Ts);
		}
	}
}

Eigen::MatrixXd SystemManager::SystemData::getMatrixSensor(double Ts, System::UpdateType type, System::InputType inType) const {
	if (type == System::MEASUREMENTUPDATE && !available())
		return Eigen::MatrixXd(0, num(System::getInputValueType(System::MEASUREMENTUPDATE, inType)));
	switch (type) {
	case System::TIMEUPDATE:
		switch (inType) {
		case System::STATE:
			return getSensorPtr()->getAi(Ts);
		case System::INPUT:
			return getSensorPtr()->getBi(Ts);
		}
	case System::MEASUREMENTUPDATE:
		switch (inType) {
		case System::STATE:
			return getSensorPtr()->getCi(Ts);
		case System::INPUT:
			return getSensorPtr()->getDi(Ts);
		}
	}
}

unsigned int SystemManager::_GetIndex(unsigned int ID) const {
	for (unsigned int i = 0; i < nSystems(); i++)
		if (systemList[i].getPtr()->getID() == ID)
			return i;
	throw std::runtime_error(std::string("Component not available."));
}

size_t SystemManager::nSystems() const { return systemList.size(); }

size_t SystemManager::num(SystemValueType type) const {
	if (type == STATE) return state.Length();
	size_t out = 0;
	for (size_t i = 0; i < nSystems(); i++)
		out += systemList[i].num(type);
	return out;
}

StatisticValue SystemManager::operator()(SystemValueType type) const {
	if (type == STATE)
		return state;

	size_t k = 0;
	for (unsigned int i = 0; i < nSystems(); i++)
		k += systemList[i].num(type);
	StatisticValue out(k);
	k = 0;
	for (unsigned int i = 0; i < nSystems(); i++) {
		size_t dk = systemList[i].num(type);
		if (dk>0)
			out.Insert(k, systemList[i](type));
		k += dk;
	}
	return out;
}

/* Get A,B, C,D matrices according to the available sensors*/


// Partitionate back the STATE, DISTURBANCE vectors
// OR
// the measured OUTPUT for the available (=not obsolethe) systems (sensors & basesystem)
// OR
// the noises for the basesystem and the active sensors

std::vector<Eigen::VectorXd> SystemManager::partitionate(SystemValueType type, Eigen::VectorXd value) const {
	std::vector<Eigen::VectorXd> out = std::vector<Eigen::VectorXd>();
	size_t n = 0;
	for (size_t i = 0; i<nSystems(); i++) {
		size_t d = systemList[i].num(type);
		out.push_back(value.segment(n, d));
		n += d;
	}
	return out;
}

void SystemManager::getMatrices(System::UpdateType out_, double Ts, Eigen::MatrixXd & A, Eigen::MatrixXd & B) const {
	SystemValueType outValueType = System::getOutputValueType(out_);
	SystemValueType inValueType = System::getInputValueType(out_, System::INPUT);
	Eigen::Index nx = state.Length();
	size_t n_in = 0, n_out = 0;
	for (size_t i = 0; i < nSystems(); i++) {
		n_in += systemList[i].num(inValueType);
		n_out += systemList[i].num(outValueType);
	}
	// init matrices az zero
	A = Eigen::MatrixXd::Zero(n_out, nx);
	B = Eigen::MatrixXd::Zero(n_out, n_in);
	// fill them
	// basesystem:
	size_t nx0 = systemList[0].num(STATE);
	size_t nin0 = systemList[0].num(inValueType);
	size_t nout0 = systemList[0].num(outValueType);
	A.block(0, 0, nout0, nx0) = systemList[0].getMatrixBaseSystem(Ts, out_, System::STATE);
	B.block(0, 0, nout0, nin0) = systemList[0].getMatrixBaseSystem(Ts, out_, System::INPUT);
	// sensors:
	size_t iin = nin0, iout = nout0, ix = nx0;
	for (size_t i = 1; i < nSystems(); i++) {
		size_t dx = systemList[i].num(STATE);
		size_t din = systemList[i].num(inValueType);
		size_t dout = systemList[i].num(outValueType);
		A.block(iout, 0, dout, nx0) = systemList[i].getMatrixBaseSystem(Ts, out_, System::STATE);
		B.block(iout, 0, dout, nin0) = systemList[i].getMatrixBaseSystem(Ts, out_, System::INPUT);
		A.block(iout, ix, dout, dx) = systemList[i].getMatrixSensor(Ts, out_, System::STATE);
		B.block(iout, iin, dout, din) = systemList[i].getMatrixSensor(Ts, out_, System::INPUT);
		iout += dout;
		iin += din;
		ix += dx;
	}
}

void SystemManager::saveMeasurement(unsigned int ID, Eigen::VectorXd value) { SystemByID(ID).set(value,SystemValueType::OUTPUT); }

SystemManager::SystemManager(SystemData data, StatisticValue state_) :
	systemList(SystemList()), state(state_), ID(getUID()) {
		systemList.push_back(data);
		unsigned int systemID = data.getPtr()->getID();
		// set callback
		data.getPtr()->AddCallback([this, systemID](Eigen::VectorXd value, EmptyClass) {
			saveMeasurement(systemID,value); }, ID);
}

SystemManager::~SystemManager() {
	for (unsigned int i = 0; i < systemList.size(); i++)
		systemList[i].getPtr()->DeleteCallback(ID);
}

void SystemManager::AddSensor(SystemData sensorData, StatisticValue sensorState) {
	if (sensorData.getSensorPtr()->isCompatible(systemList[0].getBaseSystemPtr())) {
		//Add to the list
		systemList.push_back(sensorData);
		// Add the initial state values and variances to the state/variance matrix
		state.Add(sensorState);
		// Set callbacks
		unsigned int sensorID = sensorData.getPtr()->getID();
		sensorData.getPtr()->AddCallback([this, sensorID](Eigen::VectorXd value,
			EmptyClass) { this->saveMeasurement(sensorID, std::move(value)); }, ID);
	}
	else throw std::runtime_error(std::string("Not compatible sensor tried to be added!\n"));
}
