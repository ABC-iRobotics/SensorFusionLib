#include "SystemManager.h"

SystemManager::SystemData::SystemData(StatisticValue noise_, StatisticValue disturbance_,
	Eigen::VectorXd measurement_, MeasurementStatus measStatus_) :
	noise(noise_), measurement(measurement_), disturbance(disturbance_), measStatus(measStatus_) {}

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
	if (available() || type==STATE || type==DISTURBANCE || (isBaseSystem() && type==NOISE))
		return getPtr()->getNumOf(type);
	else return 0;
}

size_t SystemManager::SystemData::num0(SystemValueType type) const { return getPtr()->getNumOf(type); }

// return length of the given value

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

int SystemManager::_GetIndex(unsigned int ID) const {
	if (ID == baseSystem.getPtr()->getID())
		return -1;
	for (unsigned int i = 0; i < nSensors(); i++)
		if (sensorList[i].getPtr()->getID() == ID)
			return i;
	throw std::runtime_error(std::string("Component not available."));
}

size_t SystemManager::nSensors() const { return sensorList.size(); }

size_t SystemManager::num(SystemValueType type) const {
	if (type == STATE) return state.Length();
	size_t out = baseSystem.num(type);
	for (size_t i = 0; i < nSensors(); i++)
		out += sensorList[i].num(type);
	return out;
}

SystemManager::SensorData SystemManager::Sensor(size_t index) const { return sensorList[index]; }

SystemManager::SensorData & SystemManager::Sensor(size_t index) { return sensorList[index]; }

const SystemManager::SystemData * SystemManager::SystemByID(unsigned int ID) const {
	int index = _GetIndex(ID);
	if (index == -1) return &baseSystem;
	return &(sensorList[index]);
}

SystemManager::SystemData * SystemManager::SystemByID(unsigned int ID) {
	int index = _GetIndex(ID);
	if (index == -1) return &baseSystem;
	return &(sensorList[index]);
}

StatisticValue SystemManager::operator()(SystemValueType type) const {
	if (type == STATE)
		return state;
	size_t k = baseSystem.num(type);
	for (unsigned int i = 0; i < nSensors(); i++)
		k += sensorList[i].num(type);
	StatisticValue out(k);
	out.Insert(0, baseSystem(type));
	k = baseSystem.num(type);
	for (unsigned int i = 0; i < nSensors(); i++) {
		size_t dk = sensorList[i].num(type);
		if (dk>0)
			out.Insert(k, sensorList[i](type));
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
	size_t n = baseSystem.num(type);
	out.push_back(value.segment(0, n));
	for (size_t i = 0; i<nSensors(); i++) {
		size_t d = sensorList[i].num(type);
		out.push_back(value.segment(n, d));
		n += d;
	}
	return out;
}

void SystemManager::getMatrices(System::UpdateType out_, double Ts, Eigen::MatrixXd & A, Eigen::MatrixXd & B) const {
	SystemValueType outValueType = System::getOutputValueType(out_);
	SystemValueType inValueType = System::getInputValueType(out_, System::INPUT);
	Eigen::Index nx = state.Length();
	size_t n_in = baseSystem.num(inValueType);
	size_t n_out = baseSystem.num(outValueType);
	for (size_t i = 0; i < nSensors(); i++) {
		n_in += sensorList[i].num(inValueType);
		n_out += sensorList[i].num(outValueType);
	}
	// init matrices az zero
	A = Eigen::MatrixXd::Zero(n_out, nx);
	B = Eigen::MatrixXd::Zero(n_out, n_in);
	// fill them
	// basesystem:
	size_t nx0 = baseSystem.num(STATE);
	size_t nin0 = baseSystem.num(inValueType);
	size_t nout0 = baseSystem.num(outValueType);
	A.block(0, 0, nout0, nx0) = baseSystem.getMatrix(Ts, out_, System::STATE);
	B.block(0, 0, nout0, nin0) = baseSystem.getMatrix(Ts, out_, System::INPUT);
	// sensors:
	size_t iin = nin0, iout = nout0, ix = nx0;
	for (size_t i = 0; i < nSensors(); i++) {
		size_t dx = sensorList[i].num(STATE);
		size_t din = sensorList[i].num(inValueType);
		size_t dout = sensorList[i].num(outValueType);
		A.block(iout, 0, dout, nx0) = sensorList[i].getMatrixBaseSystem(Ts, out_, System::STATE);
		B.block(iout, 0, dout, nin0) = sensorList[i].getMatrixBaseSystem(Ts, out_, System::INPUT);
		A.block(iout, ix, dout, dx) = sensorList[i].getMatrixSensor(Ts, out_, System::STATE);
		B.block(iout, iin, dout, din) = sensorList[i].getMatrixSensor(Ts, out_, System::INPUT);
		iout += dout;
		iin += din;
		ix += dx;
	}
}

// could be faster....

Eigen::VectorXd SystemManager::EvalNonLinPart(double Ts, System::UpdateType outType, Eigen::VectorXd state, Eigen::VectorXd in) const {
	SystemValueType intype = System::getInputValueType(outType, System::INPUT);
	SystemValueType outtype = System::getOutputValueType(outType);
	unsigned int n_out = num(outtype);
	// Partitionate vectors
	std::vector<Eigen::VectorXd> states = partitionate(STATE, state);
	std::vector<Eigen::VectorXd> ins = partitionate(intype, in);
	// Return value
	Eigen::VectorXd out = Eigen::VectorXd(n_out);
	// Call the functions
	size_t n;
	if (outType == System::TIMEUPDATE || baseSystem.available()) {
		n = baseSystem.num(outtype);
		out.segment(0, n) = baseSystem.getBaseSystemPtr()->genNonlinearPart(outType, Ts, states[0], ins[0]);
	}
	else n = 0;
	for (size_t i = 0; i < nSensors(); i++)
		if (outType == System::TIMEUPDATE || sensorList[i].available()) {
			size_t d = sensorList[i].num(outtype);
			out.segment(n, d) = sensorList[i].getSensorPtr()->genNonlinearPart(outType, Ts, states[0], ins[0], states[i], ins[i]);
			n += d;
		}
	return out;
}

void SystemManager::saveMeasurement(unsigned int ID, Eigen::VectorXd value) { SystemByID(ID)->set(value,SystemValueType::OUTPUT); }

SystemManager::SystemManager(BaseSystemData data, StatisticValue state_) :
	sensorList(SensorList()), state(state_), ID(getUID()), baseSystem(data) {
		unsigned int systemID = data.getPtr()->getID();
		// set callback
		data.getPtr()->AddCallback([this, systemID](Eigen::VectorXd value, EmptyClass) {
			saveMeasurement(systemID,value); }, ID);
}

SystemManager::~SystemManager() {
	baseSystem.getPtr()->DeleteCallback(ID);
	for (unsigned int i = 0; i < sensorList.size(); i++)
		sensorList[i].getPtr()->DeleteCallback(ID);
}

void SystemManager::AddSensor(SensorData sensorData, StatisticValue sensorState) {
	if (sensorData.getSensorPtr()->isCompatible(baseSystem.getBaseSystemPtr())) {
		//Add to the list
		sensorList.push_back(sensorData);
		// Add the initial state values and variances to the state/variance matrix
		state.Add(sensorState);
		// Set callbacks
		unsigned int sensorID = sensorData.getPtr()->getID();
		sensorData.getPtr()->AddCallback([this, sensorID](Eigen::VectorXd value,
			EmptyClass) { this->saveMeasurement(sensorID, std::move(value)); }, ID);
	}
	else throw std::runtime_error(std::string("Not compatible sensor tried to be added!\n"));
}

SystemManager::BaseSystemData::BaseSystemData(BaseSystem::BaseSystemPtr ptr_, StatisticValue noise_,
	StatisticValue disturbance_, Eigen::VectorXd measurement_, MeasurementStatus measStatus_) : ptr(ptr_),
	SystemData(noise_, disturbance_, measurement_, measStatus_) {}

Eigen::VectorXi SystemManager::BaseSystemData::dep(System::UpdateType outType, System::InputType type) const {
	if (outType == System::TIMEUPDATE || available())
		return ptr->genNonlinearDependency(outType, type);
	else return Eigen::VectorXi::Zero(num0(System::getInputValueType(outType, type)));
}

Eigen::MatrixXd SystemManager::BaseSystemData::getMatrix(double Ts, System::UpdateType type, System::InputType inType) const {
	if (type == System::MEASUREMENTUPDATE && !available()) {
		switch (inType) {
		case System::STATE:
			return Eigen::MatrixXd(0, ptr->getNumOfStates());
		case System::INPUT:
			return Eigen::MatrixXd(0, ptr->getNumOfNoises());
		}
	}
	switch (type) {
	case System::TIMEUPDATE:
		switch (inType) {
		case System::STATE:
			return ptr->getA(Ts);
		case System::INPUT:
			return ptr->getB(Ts);
		}
	case System::MEASUREMENTUPDATE:
		switch (inType) {
		case System::STATE:
			return ptr->getC(Ts);
		case System::INPUT:
			return ptr->getD(Ts);
		}
	}
}

BaseSystem::BaseSystemPtr SystemManager::BaseSystemData::getBaseSystemPtr() const { return ptr; }

System::SystemPtr SystemManager::BaseSystemData::getPtr() const { return ptr; }

bool SystemManager::BaseSystemData::isBaseSystem() const { return true; }

SystemManager::SensorData::SensorData(Sensor::SensorPtr ptr_, StatisticValue noise_,
	StatisticValue disturbance_, Eigen::VectorXd measurement_, MeasurementStatus measStatus_) : ptr(ptr_),
	SystemData(noise_, disturbance_, measurement_, measStatus_) {}

Eigen::VectorXi SystemManager::SensorData::depSensor(System::UpdateType outType, System::InputType type) const {
	if (outType == System::TIMEUPDATE || available())
		return ptr->genNonlinearSensorDependency(outType, type);
	else return Eigen::VectorXi::Zero(num0(System::getInputValueType(outType, type)));
}

Eigen::VectorXi SystemManager::SensorData::depBaseSystem(System::UpdateType outType, System::InputType type) const {
	if (outType == System::TIMEUPDATE || available())
		return ptr->genNonlinearBaseSystemDependency(outType, type);
	else return Eigen::VectorXi::Zero(num0(System::getInputValueType(outType, type)));
}

Eigen::MatrixXd SystemManager::SensorData::getMatrixBaseSystem(double Ts, System::UpdateType type, System::InputType inType) const {
	if (type == System::MEASUREMENTUPDATE && !available()) {
		switch (inType) {
		case System::STATE:
			return Eigen::MatrixXd(0, ptr->getNumOfBaseSystemStates());
		case System::INPUT:
			return Eigen::MatrixXd(0, ptr->getNumOfBaseSystemNoises());
		}
	}
	switch (type) {
	case System::TIMEUPDATE:
		switch (inType) {
		case System::STATE:
			return ptr->getA0(Ts);
		case System::INPUT:
			return ptr->getB0(Ts);
		}
	case System::MEASUREMENTUPDATE:
		switch (inType) {
		case System::STATE:
			return ptr->getC0(Ts);
		case System::INPUT:
			return ptr->getD0(Ts);
		}
	}
}

Eigen::MatrixXd SystemManager::SensorData::getMatrixSensor(double Ts, System::UpdateType type, System::InputType inType) const {
	if (type == System::MEASUREMENTUPDATE && !available())
		return Eigen::MatrixXd(0, num(System::getInputValueType(System::MEASUREMENTUPDATE, inType)));
	switch (type) {
	case System::TIMEUPDATE:
		switch (inType) {
		case System::STATE:
			return ptr->getAi(Ts);
		case System::INPUT:
			return ptr->getBi(Ts);
		}
	case System::MEASUREMENTUPDATE:
		switch (inType) {
		case System::STATE:
			return ptr->getCi(Ts);
		case System::INPUT:
			return ptr->getDi(Ts);
		}
	}
}

Sensor::SensorPtr SystemManager::SensorData::getSensorPtr() const { return ptr; }

System::SystemPtr SystemManager::SensorData::getPtr() const { return ptr; }

bool SystemManager::SensorData::isBaseSystem() const { return false; }
