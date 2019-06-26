#include "SystemManager.h"
#include "PartialCholevski.h"

SystemManager::SystemData::SystemData(const StatisticValue& noise_,
	const StatisticValue& disturbance_, const Eigen::VectorXd& measurement_,
	MeasurementStatus measStatus_) :
	noise(noise_), measurement(measurement_), disturbance(disturbance_), measStatus(measStatus_) {}

StatisticValue SystemManager::SystemData::operator()(DataType type, bool forcedOutput) const {
	if (type== DataType::DISTURBANCE) return disturbance;
	if (!(available() || forcedOutput)) return Eigen::VectorXd(0);
	switch (type) {
	case DataType::NOISE:
		return noise;
	case DataType::OUTPUT:
		return measurement;
	default:
		throw std::runtime_error(std::string("SystemManager::SystemData::operator() Only NOISE, DISTURBANCE and OUTPUT are available!"));
	}
}

size_t SystemManager::SystemData::num(DataType type, bool forcedOutput) const
{
	if (available() || forcedOutput || type==STATE || type==DISTURBANCE || (isBaseSystem() && type==NOISE))
		return getPtr()->getNumOf(type);
	else return 0;
}

// return length of the given value

void SystemManager::SystemData::setValue(const Eigen::VectorXd & value, DataType type) {
	switch (type) {
	case DataType::NOISE:
		noise.vector = value;
		break;
	case DataType::DISTURBANCE:
		disturbance.vector = value;
		break;
	case DataType::OUTPUT:
		measurement = value;
		if (measStatus == OBSOLETHE)
			measStatus = UPTODATE;
		break;
	default:
		throw std::runtime_error(std::string("SystemData::setValue(): Wrong argument\n"));
	}
}

void SystemManager::SystemData::setVariance(const Eigen::MatrixXd & value, DataType type) {
	switch (type) {
	case DataType::NOISE:
		noise.variance = value;
		break;
	case DataType::DISTURBANCE:
		disturbance.variance = value;
		break;
	default:
		throw std::runtime_error(std::string("SystemData::setValue(): Wrong argument\n"));
	}
}

// set the given value

Eigen::VectorXd SystemManager::SystemData::getValue(DataType type) const {
	switch (type) {
	case DataType::NOISE:
		return noise.vector;
		break;
	case DataType::DISTURBANCE:
		return disturbance.vector;
	case DataType::OUTPUT:
		return measurement;
	default:
		throw std::runtime_error(std::string("SystemData::setValue(): Wrong argument\n"));
	}
}

Eigen::MatrixXd SystemManager::SystemData::getVariance(DataType type) const {
	switch (type) {
	case DataType::NOISE:
		return noise.variance;
		break;
	case DataType::DISTURBANCE:
		return disturbance.variance;
	default:
		throw std::runtime_error(std::string("SystemData::setValue(): Wrong argument\n"));
	}
}

void SystemManager::SystemData::resetMeasurement() {
	if (measStatus == UPTODATE)
		measStatus = OBSOLETHE;
}

bool SystemManager::SystemData::available() const { return measStatus != OBSOLETHE; }

// returns if is measurement available

SystemManager::Partitioner SystemManager::getPartitioner(bool forcedOutput) {
	Partitioner p(nSensors() + 1);
	p.nx[0] = BaseSystem().num(DataType::STATE, forcedOutput);
	p.ny[0] = BaseSystem().num(DataType::OUTPUT, forcedOutput);
	p.nw[0] = BaseSystem().num(DataType::DISTURBANCE, forcedOutput);
	p.nv[0] = BaseSystem().num(DataType::NOISE, forcedOutput);
	for (size_t n = 0; n < nSensors(); n++) {
		p.nx[n + 1] = Sensor(n).num(DataType::STATE, forcedOutput);
		p.ny[n + 1] = Sensor(n).num(DataType::OUTPUT, forcedOutput);
		p.nw[n + 1] = Sensor(n).num(DataType::DISTURBANCE, forcedOutput);
		p.nv[n + 1] = Sensor(n).num(DataType::NOISE, forcedOutput);
	}
	return p;
}

bool SystemManager::available(int index) const {
	if (index == -1)
		return BaseSystem().available();
	return Sensor(index).available();
}

int SystemManager::_GetIndex(unsigned int ID) const {
	if (ID == baseSystem.getPtr()->getID())
		return -1;
	for (unsigned int i = 0; i < nSensors(); i++)
		if (sensorList[i].getPtr()->getID() == ID)
			return i;
	throw std::runtime_error(std::string("SystemManager::_GetIndex(): unknown ID"));
}

// returns -1 for the basesystem!

SystemManager::BaseSystemData & SystemManager::BaseSystem() { return baseSystem; }

const SystemManager::BaseSystemData & SystemManager::BaseSystem() const { return baseSystem; }

// Add a sensor

void SystemManager::AddSensor(const SensorData & sensorData, const StatisticValue & sensorState) {
	if (sensorData.getSensorPtr()->isCompatible(baseSystem.getBaseSystemPtr())) {
		//Add to the list
		sensorList.push_back(sensorData);
		// Add the initial state values and variances to the state/variance matrix
		state.Add(sensorState);
		// Set callbacks
		unsigned int sensorID = sensorData.getPtr()->getID();
		sensorData.getPtr()->AddCallback([this, sensorID](SystemCallData call) {
			_setProperty(sensorID, call);
		}, ID);
	}
	else throw std::runtime_error(std::string("SystemManager::AddSensor(): Not compatible sensor tried to be added!\n"));
}

size_t SystemManager::nSensors() const { return sensorList.size(); }

size_t SystemManager::num(DataType type, bool forcedOutput) const {
	if (type == STATE) return state.Length();
	size_t out = baseSystem.num(type, forcedOutput);
	for (size_t i = 0; i < nSensors(); i++)
		out += sensorList[i].num(type, forcedOutput);
	return out;
}

Eigen::VectorXi SystemManager::dep(TimeUpdateType outType, VariableType inType, bool forcedOutput) const {
	DataType type = System::getInputValueType(outType, inType);
	Eigen::Index n = num(type, forcedOutput);
	// Get nonlinear dependencies
	Eigen::VectorXi dep = Eigen::VectorXi(n);
	// Sum dependencies from basesystem properties
	Eigen::VectorXi baseSystemDep = baseSystem.dep(outType, inType, forcedOutput);
	for (size_t i = 0; i < nSensors(); i++) {
		Eigen::VectorXi temp = sensorList[i].depBaseSystem(outType, inType, forcedOutput);
		for (Eigen::Index j = 0; j < temp.size(); j++)
			if (temp[j] == 1) baseSystemDep[j] = 1;
	}
	// Concatenate dep vectors
	Eigen::Index j = baseSystemDep.size();
	dep.segment(0, j) = baseSystemDep;
	for (size_t i = 0; i < nSensors(); i++) {
		Eigen::VectorXi temp = sensorList[i].depSensor(outType, inType, forcedOutput);
		Eigen::Index d = temp.size();
		dep.segment(j, d) = temp;
		j += d;
	}
	return dep;
}

const SystemManager::SensorData& SystemManager::Sensor(size_t index) const { return sensorList[index]; }

void SystemManager::_setProperty(int systemID, SystemCallData call) {
	SystemData* ptr = this->SystemByID(systemID);
	if (call.valueType == VALUE)
		ptr->setValue(call.value, call.signalType);
	else
		ptr->setVariance(call.variance, call.signalType);
	if (call.signalType == DataType::OUTPUT)
		Call(FilterCallData(call.value, ptr->getPtr(), this->t, call.signalType, FilterCallData::MEASUREMENT));
}

SystemManager::SensorData & SystemManager::Sensor(size_t index) { return sensorList[index]; }

StatisticValue & SystemManager::State() { return state; }

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

StatisticValue SystemManager::operator()(DataType type, bool forcedOutput) const {
	if (type == STATE)
		return state;
	size_t k = baseSystem.num(type, forcedOutput);
	for (unsigned int i = 0; i < nSensors(); i++)
		k += sensorList[i].num(type, forcedOutput);
	StatisticValue out(k);
	out.Insert(0, baseSystem(type));
	k = baseSystem.num(type, forcedOutput);
	for (unsigned int i = 0; i < nSensors(); i++) {
		size_t dk = sensorList[i].num(type, forcedOutput);
		if (dk>0)
			out.Insert(k, sensorList[i](type, forcedOutput));
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

std::vector<Eigen::VectorXd> SystemManager::partitionate(DataType type, const Eigen::VectorXd& value, bool forcedOutput) const {
	std::vector<Eigen::VectorXd> out = std::vector<Eigen::VectorXd>();
	size_t n = baseSystem.num(type, forcedOutput);
	out.push_back(value.segment(0, n));
	for (size_t i = 0; i<nSensors(); i++) {
		size_t d = sensorList[i].num(type, forcedOutput);
		out.push_back(value.segment(n, d));
		n += d;
	}
	return out;
}

std::vector<StatisticValue> SystemManager::partitionateWithStatistic(DataType type, const StatisticValue& value, bool forcedOutput) const {
	std::vector<StatisticValue> out = std::vector<StatisticValue>();
	size_t n = baseSystem.num(type, forcedOutput);
	out.push_back(value.GetPart(0, n));
	for (size_t i = 0; i < nSensors(); i++) {
		size_t d = sensorList[i].num(type, forcedOutput);
		out.push_back(value.GetPart(n, d));
		n += d;
	}
	return out;
}

void SystemManager::getMatrices(TimeUpdateType out_, double Ts, Eigen::MatrixXd & A,
	Eigen::MatrixXd & B, bool forcedOutput) const {
	DataType outValueType = System::getOutputValueType(out_);
	DataType inValueType = System::getInputValueType(out_, VAR_EXTERNAL);
	Eigen::Index nx = state.Length();
	size_t n_in = baseSystem.num(inValueType, forcedOutput);
	size_t n_out = baseSystem.num(outValueType, forcedOutput);
	for (size_t i = 0; i < nSensors(); i++) {
		n_in += sensorList[i].num(inValueType, forcedOutput);
		n_out += sensorList[i].num(outValueType, forcedOutput);
	}
	// init matrices az zero
	A = Eigen::MatrixXd::Zero(n_out, nx);
	B = Eigen::MatrixXd::Zero(n_out, n_in);
	// fill them
	// basesystem:
	size_t nx0 = baseSystem.num(STATE, forcedOutput);
	size_t nin0 = baseSystem.num(inValueType, forcedOutput);
	size_t nout0 = baseSystem.num(outValueType, forcedOutput);
	A.block(0, 0, nout0, nx0) = baseSystem.getMatrix(Ts, out_, VAR_STATE, forcedOutput);
	B.block(0, 0, nout0, nin0) = baseSystem.getMatrix(Ts, out_, VAR_EXTERNAL, forcedOutput);
	// sensors:
	size_t iin = nin0, iout = nout0, ix = nx0;
	for (size_t i = 0; i < nSensors(); i++) {
		size_t dx = sensorList[i].num(STATE, forcedOutput);
		size_t din = sensorList[i].num(inValueType, forcedOutput);
		size_t dout = sensorList[i].num(outValueType, forcedOutput);
		A.block(iout, 0, dout, nx0) = sensorList[i].getMatrixBaseSystem(Ts, out_, VAR_STATE, forcedOutput);
		B.block(iout, 0, dout, nin0) = sensorList[i].getMatrixBaseSystem(Ts, out_, VAR_EXTERNAL, forcedOutput);
		A.block(iout, ix, dout, dx) = sensorList[i].getMatrixSensor(Ts, out_, VAR_STATE, forcedOutput);
		B.block(iout, iin, dout, din) = sensorList[i].getMatrixSensor(Ts, out_, VAR_EXTERNAL, forcedOutput);
		iout += dout;
		iin += din;
		ix += dx;
	}
}

// could be faster....

Eigen::VectorXd SystemManager::EvalNonLinPart(double Ts,
 TimeUpdateType outType, const Eigen::VectorXd& state, const Eigen::VectorXd& in, bool forcedOutput) const {
	DataType intype = System::getInputValueType(outType, VAR_EXTERNAL);
	DataType outtype = System::getOutputValueType(outType);
	size_t n_out = num(outtype, forcedOutput);
	// Partitionate vectors
	std::vector<Eigen::VectorXd> states = partitionate(STATE, state, forcedOutput);
	std::vector<Eigen::VectorXd> ins = partitionate(intype, in, forcedOutput);
	// Return value
	Eigen::VectorXd out = Eigen::VectorXd(n_out);
	// Call the functions
	size_t n;
	if (outType == STATE_UPDATE || baseSystem.available() || forcedOutput) {
		n = baseSystem.num(outtype, forcedOutput);
		out.segment(0, n) = baseSystem.getBaseSystemPtr()->genNonlinearPart(outType, Ts, states[0], ins[0]);
	}
	else n = 0;
	for (size_t i = 0; i < nSensors(); i++)
		if (outType == STATE_UPDATE || sensorList[i].available() || forcedOutput) {
			size_t d = sensorList[i].num(outtype, forcedOutput);
			out.segment(n, d) = sensorList[i].getSensorPtr()->genNonlinearPart(outType, Ts, states[0], ins[0], states[i+1], ins[i+1]);
			n += d;
		}
	return out;
}

StatisticValue SystemManager::Eval(TimeUpdateType outType, double Ts, const StatisticValue& state_,
	const StatisticValue& in, Eigen::MatrixXd & S_out_x, Eigen::MatrixXd& S_out_in, bool forcedOutput) const {
	DataType inType = System::getInputValueType(outType, VAR_EXTERNAL);
	Eigen::Index nX = num(STATE, forcedOutput);
	Eigen::Index nIn = num(inType, forcedOutput);
	// Get nonlinear dependencies
	Eigen::VectorXi stateDep = dep(outType, VAR_STATE, forcedOutput);
	Eigen::VectorXi inDep = dep(outType, VAR_EXTERNAL, forcedOutput);
	size_t nOut = num(System::getOutputValueType(outType), forcedOutput);
	// 
	Eigen::VectorXd z;
	Eigen::MatrixXd Sz;
	Eigen::MatrixXd Szx;
	Eigen::MatrixXd Szw;
	if (stateDep.sum() + inDep.sum() == 0) {
		z = Eigen::VectorXd::Zero(nOut);
		Sz = Eigen::MatrixXd::Zero(nOut, nOut);
		Szx = Eigen::MatrixXd::Zero(nOut, nX);
		Szw = Eigen::MatrixXd::Zero(nOut, nIn);
	}
	else {
		// Sigma values by applying partial chol
		Eigen::MatrixXd dX = PartialChol(state_.variance, stateDep);
		Eigen::MatrixXd dIn;
		if (in.isIndependent) {
			dIn = Eigen::MatrixXd::Zero(nIn, inDep.sum());
			unsigned int j = 0;
			for (unsigned int i = 0; i < nIn; i++)
				if (inDep[i] == 1) {
					dIn(i, j) = sqrt(in.variance(i, i));
					j++;
				}
		}
		else
			dIn = PartialChol(in.variance, inDep);
		Eigen::Index nNL_X = dX.cols();
		Eigen::Index nNL_In = dIn.cols();
		Eigen::Index nNL = nNL_X + nNL_In;
		// Constants for UT
		double alpha = 0.7;
		double beta = 2.;
		double kappa = 1e-8;
		double tau2 = alpha * alpha * (kappa + (double)nNL);
		double tau = sqrt(tau2);
		dX *= tau;
		dIn *= tau;

		// Mean value
		Eigen::VectorXd z0 = EvalNonLinPart(Ts, outType, state_.vector, in.vector, forcedOutput);
		std::vector<Eigen::VectorXd> z_x = std::vector<Eigen::VectorXd>();

		for (Eigen::Index i = 0; i < nNL_X; i++) {
			z_x.push_back(EvalNonLinPart(Ts, outType, state_.vector + dX.col(i), in.vector, forcedOutput));
			//std::cout << "x: \n" << state_.vector + dX.col(i) << "\nz:\n" << z_x[i * 2] << std::endl;
			z_x.push_back(EvalNonLinPart(Ts, outType, state_.vector - dX.col(i), in.vector, forcedOutput));
			//std::cout << "x: \n" << state_.vector - dX.col(i) << "\nz:\n" << z_x[i * 2+1] << std::endl;
		}
		std::vector<Eigen::VectorXd> z_w = std::vector<Eigen::VectorXd>();
		for (Eigen::Index i = 0; i < nNL_In; i++) {
			z_w.push_back(EvalNonLinPart(Ts, outType, state_.vector, in.vector + dIn.col(i), forcedOutput));
			z_w.push_back(EvalNonLinPart(Ts, outType, state_.vector, in.vector - dIn.col(i), forcedOutput));
		}
		z = (1. - (double)nNL / tau2 ) * z0;
		for (int i = 0; i < 2 * nNL_X; i++)
			z += z_x[i] / 2. / tau2;
		for (int i = 0; i < 2 * nNL_In; i++)
			z += z_w[i] / 2. / tau2;

		Sz = ((tau2 - (double)nNL) / tau2 + 1. + beta - alpha * alpha) * (z0 - z) * (z0 - z).transpose();
		Szx = Eigen::MatrixXd::Zero(nOut, nX);
		Szw = Eigen::MatrixXd::Zero(nOut, nIn);
		Eigen::VectorXd temp;
		for (unsigned int i = 0; i < nNL_X; i++) {
			temp = z_x[i] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			temp = z_x[i + nNL_X] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			Szx += (z_x[2 * i] - z_x[2 * i + 1])*dX.col(i).transpose() / 2. / tau2;
		}
		for (int i = 0; i < nNL_In; i++) {
			temp = z_w[i] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			temp = z_w[i + nNL_In] - z;
			Sz += temp * temp.transpose() / 2. / tau2;
			Szw += (z_w[2 * i] - z_w[2 * i + 1])*dIn.col(i).transpose() / 2. / tau2;
		}
	}
	// Get coefficient matrices
	Eigen::MatrixXd A, B;
	getMatrices(outType, Ts, A, B, forcedOutput);

	Eigen::VectorXd y = A * state_.vector + B * in.vector + z;
	S_out_x = A * state_.variance + Szx;
	S_out_in = B * in.variance + Szw;
	Eigen::MatrixXd Sy = S_out_x * A.transpose() + S_out_in * B.transpose() +
		Sz + A * Szx.transpose() + B * Szw.transpose();

	return StatisticValue(y, Sy);
}

SystemManager::SystemManager(const BaseSystemData& data, const StatisticValue& state_) :
	sensorList(SensorList()), state(state_), ID(getUID()), baseSystem(data), t(0) {
		unsigned int systemID = data.getPtr()->getID();
		// set callback
		data.getPtr()->AddCallback([this, systemID](SystemCallData call) {
			_setProperty(systemID, call);
		}, ID);
}

SystemManager::~SystemManager() {
	baseSystem.getPtr()->DeleteCallback(ID);
	for (size_t i = 0; i < sensorList.size(); i++)
		sensorList[i].getPtr()->DeleteCallback(ID);
}

std::ostream & SystemManager::print(std::ostream & stream) const {
	std::vector<Eigen::VectorXd> states = partitionate(STATE, state.vector);
	Eigen::MatrixXd S1, S2;
	StatisticValue output = Eval(OUTPUT_UPDATE, 0.001, state, (*this)(NOISE, true), S1, S2, true);
	std::vector<Eigen::VectorXd> outputs = partitionate(OUTPUT, output.vector, true);

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
		printSystem(stream, &sensorList[sensor_i], states[sensor_i + 1], outputs[sensor_i + 1]);
	}
	// STATE variances
	stream << "Variance matrix of the state:\n";
	std::vector<size_t> statesizes = std::vector<size_t>();
	statesizes.push_back(baseSystem.num(STATE));
	for (size_t i = 0; i < nSensors(); i++)
		statesizes.push_back(sensorList[i].num(STATE));
	unsigned int a = 0, b = 0;
	for (unsigned int i = 0; i < statesizes.size(); i++) {
		for (unsigned int di = 0; di < statesizes[i]; di++) {
			for (unsigned int j = 0; j < statesizes.size(); j++) {
				for (unsigned int dj = 0; dj < statesizes[j]; dj++) {
					stream << state.variance(a, b) << " ";
					b++;
				}
				if (j + 1 < statesizes.size())
					stream << "| ";
			}
			stream << std::endl;
			b = 0;
			a++;
		}
		if (i + 1 < statesizes.size())
			for (unsigned int k = 0; k < 40; k++)
				stream << "-";
		stream << std::endl;
	}
	return stream;
}

void SystemManager::resetMeasurement() {
	baseSystem.resetMeasurement();
	for (int i = 0; i < nSensors(); i++)
		sensorList[i].resetMeasurement();
}

void SystemManager::PredictionDone(const StatisticValue& state, const StatisticValue& output) const {
	std::vector<StatisticValue> vState = partitionateWithStatistic(STATE, state);
	std::vector<StatisticValue> vOutput = partitionateWithStatistic(OUTPUT, output);
	Call(FilterCallData(vState[0], baseSystem.getPtr(), t, STATE, FilterCallData::PREDICTION));
	Call(FilterCallData(baseSystem(DISTURBANCE), baseSystem.getPtr(), t, DISTURBANCE, FilterCallData::PREDICTION));
	if (baseSystem.available()) {
		Call(FilterCallData(vOutput[0], baseSystem.getPtr(), t, OUTPUT, FilterCallData::PREDICTION));
		Call(FilterCallData(baseSystem(NOISE), baseSystem.getPtr(), t, NOISE, FilterCallData::PREDICTION));
	}
	for (int i = 0; i < nSensors(); i++) {
		Call(FilterCallData(vState[i + 1], sensorList[i].getPtr(), t, STATE, FilterCallData::PREDICTION));
		Call(FilterCallData(sensorList[i](DISTURBANCE), sensorList[i].getPtr(), t, DISTURBANCE, FilterCallData::PREDICTION));
		if (sensorList[i].available()) {
			Call(FilterCallData(vOutput[i + 1], sensorList[i].getPtr(), t, OUTPUT, FilterCallData::PREDICTION));
			Call(FilterCallData(sensorList[i](NOISE), sensorList[i].getPtr(), t, NOISE, FilterCallData::PREDICTION));
		}
	}
}

void SystemManager::FilteringDone(const StatisticValue& state) const {
	std::vector<Eigen::VectorXd> vState = partitionate(STATE, state.vector);
	Call(FilterCallData(vState[0], baseSystem.getPtr(), t, STATE, FilterCallData::FILTERING));
	for (int i = 0; i < nSensors(); i++)
		Call(FilterCallData(vState[i + 1], sensorList[i].getPtr(), t, STATE, FilterCallData::FILTERING));
}

void SystemManager::StepClock(double dt) { t += dt; }

SystemManager::BaseSystemData::BaseSystemData(BaseSystem::BaseSystemPtr ptr_,
	const StatisticValue& noise_, const StatisticValue& disturbance_, const Eigen::VectorXd& measurement_,
	MeasurementStatus measStatus_) : ptr(ptr_),
	SystemData(noise_, disturbance_, measurement_, measStatus_) {
	if (noise_.Length() != ptr_->getNumOfNoises() ||
		disturbance_.Length() != ptr_->getNumOfDisturbances() ||
		(measStatus_ != OBSOLETHE && measurement_.size() != ptr_->getNumOfOutputs()))
		throw std::runtime_error(std::string("BaseSystemData::BaseSystemData Wrong argument sizes"));
}

Eigen::VectorXi SystemManager::BaseSystemData::dep(TimeUpdateType outType, VariableType type, bool forcedOutput) const {
	if (outType == STATE_UPDATE || available() || forcedOutput)
		return ptr->genNonlinearDependency(outType, type);
	else return Eigen::VectorXi::Zero(num(System::getInputValueType(outType, type), true));
}

Eigen::MatrixXd SystemManager::BaseSystemData::getMatrix(double Ts, TimeUpdateType type, VariableType inType, bool forcedOutput) const {
	if (type == OUTPUT_UPDATE && !(available() || forcedOutput)) {
		switch (inType) {
		case VAR_STATE:
			return Eigen::MatrixXd(0, ptr->getNumOfStates());
		case VAR_EXTERNAL:
			return Eigen::MatrixXd(0, ptr->getNumOfNoises());
		}
	}
	switch (type) {
	case STATE_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return ptr->getA(Ts);
		case VAR_EXTERNAL:
			return ptr->getB(Ts);
		}
	case OUTPUT_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return ptr->getC(Ts);
		case VAR_EXTERNAL:
			return ptr->getD(Ts);
		}
	}
	throw std::runtime_error(std::string("SystemManager::BaseSystemData::getMatrix(): unknown parameters"));
}

BaseSystem::BaseSystemPtr SystemManager::BaseSystemData::getBaseSystemPtr() const { return ptr; }

System::SystemPtr SystemManager::BaseSystemData::getPtr() const { return ptr; }

bool SystemManager::BaseSystemData::isBaseSystem() const { return true; }

SystemManager::SensorData::SensorData(Sensor::SensorPtr ptr_, const StatisticValue& noise_,
	const StatisticValue& disturbance_, const Eigen::VectorXd& measurement_,
	MeasurementStatus measStatus_) : ptr(ptr_),
	SystemData(noise_, disturbance_, measurement_, measStatus_) {
	if (noise_.Length() != ptr_->getNumOfNoises() ||
		disturbance_.Length() != ptr_->getNumOfDisturbances() ||
		(measStatus_ != OBSOLETHE && measurement_.size() != ptr_->getNumOfOutputs()))
		throw std::runtime_error(std::string("SensorData::SensorData Wrong argument sizes"));
}

Eigen::VectorXi SystemManager::SensorData::depSensor(TimeUpdateType outType,
	VariableType type, bool forcedOutput) const {
	if (outType == STATE_UPDATE || available() || forcedOutput)
		return ptr->genNonlinearSensorDependency(outType, type);
	else return Eigen::VectorXi::Zero(num(System::getInputValueType(outType, type)));
}

Eigen::VectorXi SystemManager::SensorData::depBaseSystem(TimeUpdateType outType,
	VariableType type, bool forcedOutput) const {
	if (outType == STATE_UPDATE || available() || forcedOutput)
		return ptr->genNonlinearBaseSystemDependency(outType, type);
	else return Eigen::VectorXi::Zero(num(System::getInputValueType(outType, type)));
}

Eigen::MatrixXd SystemManager::SensorData::getMatrixBaseSystem(double Ts, TimeUpdateType type,
	VariableType inType, bool forcedOutput) const {
	if (type == OUTPUT_UPDATE && !(available() || forcedOutput)) {
		switch (inType) {
		case VAR_STATE:
			return Eigen::MatrixXd(0, ptr->getNumOfBaseSystemStates());
		case VAR_EXTERNAL:
			return Eigen::MatrixXd(0, ptr->getNumOfBaseSystemNoises());
		}
	}
	switch (type) {
	case STATE_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return ptr->getA0(Ts);
		case VAR_EXTERNAL:
			return ptr->getB0(Ts);
		}
	case OUTPUT_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return ptr->getC0(Ts);
		case VAR_EXTERNAL:
			return ptr->getD0(Ts);
		}
	}
	throw std::runtime_error(std::string("SystemManager::SensorData::getMatrixBaseSystem(): unknown parameters"));
}

Eigen::MatrixXd SystemManager::SensorData::getMatrixSensor(double Ts,
	TimeUpdateType type, VariableType inType, bool forcedOutput) const {
	if (type == OUTPUT_UPDATE && !(available() || forcedOutput))
		return Eigen::MatrixXd(0, num(System::getInputValueType(OUTPUT_UPDATE, inType)));
	switch (type) {
	case STATE_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return ptr->getAi(Ts);
		case VAR_EXTERNAL:
			return ptr->getBi(Ts);
		}
	case OUTPUT_UPDATE:
		switch (inType) {
		case VAR_STATE:
			return ptr->getCi(Ts);
		case VAR_EXTERNAL:
			return ptr->getDi(Ts);
		}
	}
	throw std::runtime_error(std::string("SystemManager::SensorData::getMatrixSensor(): unknown parameters"));
}

Sensor::SensorPtr SystemManager::SensorData::getSensorPtr() const { return ptr; }

System::SystemPtr SystemManager::SensorData::getPtr() const { return ptr; }

bool SystemManager::SensorData::isBaseSystem() const { return false; }

SystemManager::Partitioner::Partitioner(size_t N) : nx(std::vector<size_t>(N)),
nw(std::vector<size_t>(N)), ny(std::vector<size_t>(N)),
nv(std::vector<size_t>(N)) {}

const std::vector<size_t>& SystemManager::Partitioner::n(DataType type) const {
	switch (type) {
	case DataType::NOISE:
		return nv;
	case DataType::DISTURBANCE:
		return nw;
	case DataType::STATE:
		return nx;
	case DataType::OUTPUT:
		return ny;
	default:
		throw std::runtime_error(std::string("Partitioner::n(): Unknown argument!"));
	}
}

Eigen::VectorBlock<Eigen::VectorXd> SystemManager::Partitioner::PartValue(DataType type, Eigen::VectorXd & value, int index) const { //index=-1: basesystem, index=0 sensor0....
	auto n_ = n(type);
	//if (index == -1) return value.segment(0, n_[0]);
	size_t n0 = 0;
	for (int i = 0; i < index + 1; i++)
		n0 += n_[i];
	return value.segment(n0, n_[index + 1]);
}

Eigen::Block<Eigen::MatrixXd> SystemManager::Partitioner::PartVariance(DataType type, Eigen::MatrixXd & value, int index1, int index2) const {
	auto n_ = n(type);
	//if (index == -1) return value.segment(0, n_[0]);
	size_t n01 = 0, n02 = 0;
	for (int i = 0; i < index1 + 1; i++)
		n01 += n_[i];
	for (int i = 0; i < index2 + 1; i++)
		n02 += n_[i];
	return value.block(n01, n02, n_[index1 + 1], n_[index2 + 1]);
}

Eigen::Block<Eigen::MatrixXd> SystemManager::Partitioner::PartVariance(DataType type1, DataType type2, Eigen::MatrixXd & value, int index1, int index2) const {
	const std::vector<size_t>& n1_ = n(type1);
	const std::vector<size_t>& n2_ = n(type2);
	//if (index == -1) return value.segment(0, n_[0]);
	size_t n01 = 0, n02 = 0;
	for (int i = 0; i < index1 + 1; i++)
		n01 += n1_[i];
	for (int i = 0; i < index2 + 1; i++)
		n02 += n2_[i];
	return value.block(n01, n02, n1_[index1 + 1], n2_[index2 + 1]);
}
