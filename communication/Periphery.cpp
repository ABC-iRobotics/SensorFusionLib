#include "Periphery.h"

SF::Periphery::Periphery(const std::string& address, int hwm) {
	SetZMQOutput(address, hwm);
}

SF::Periphery::Periphery(const NetworkConfig::ConnectionData & config) {
	SetZMQOutput(config.SenderAddress(), config.hwm != -1 ? config.hwm : 10);
}

void SF::Periphery::SendValue(unsigned char sensorID, const Eigen::VectorXd & value, DataType type, Time t, OperationType source) {
	DataMsg msg(sensorID, type, source, t);
	msg.SetValueVector(value);
	ForwardDataMsg(msg, Now());
}

void SF::Periphery::SendVariance(unsigned char sensorID, const Eigen::MatrixXd & variance, DataType type, Time t, OperationType source) {
	DataMsg msg(sensorID, type, source, t);
	msg.SetVarianceMatrix(variance);
	ForwardDataMsg(msg, Now());
}

void SF::Periphery::SendValueAndVariance(unsigned char sensorID, const Eigen::VectorXd & value,
	const Eigen::MatrixXd & variance, DataType type,Time t, OperationType source) {
	DataMsg msg(sensorID, type, source, t);
	msg.SetValueVector(value);
	msg.SetVarianceMatrix(variance);
	ForwardDataMsg(msg, Now());
}
