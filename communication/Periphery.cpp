#include "Periphery.h"
#include "ZMQCommunication.h"

SF::Periphery::Periphery(const std::string& address, int hwm) :
	ptr(std::make_shared<ZMQSender>(address, hwm)) {}

void SF::Periphery::SendValue(unsigned char sensorID, const Eigen::VectorXd & value, DataType type, Time t, OperationType source) {
	DataMsg msg(sensorID, type, source, t);
	msg.SetValueVector(value);
	ptr->SendDataMsg(msg);
}

void SF::Periphery::SendVariance(unsigned char sensorID, const Eigen::MatrixXd & variance, DataType type, Time t, OperationType source) {
	DataMsg msg(sensorID, type, source, t);
	msg.SetVarianceMatrix(variance);
	ptr->SendDataMsg(msg);
}

void SF::Periphery::SendValueAndVariance(unsigned char sensorID, const Eigen::VectorXd & value,
	const Eigen::MatrixXd & variance, DataType type,Time t, OperationType source) {
	DataMsg msg(sensorID, type, source, t);
	msg.SetValueVector(value);
	msg.SetVarianceMatrix(variance);
	ptr->SendDataMsg(msg);
}
