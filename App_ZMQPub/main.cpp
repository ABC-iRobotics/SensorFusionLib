#include <thread>
#include "ZMQPublisher.h"

DataMsg msg1(int i) {
	DataMsg msg(5, STATE, SENSOR);
	msg.SetVarianceMatrix(Eigen::MatrixXd::Identity(4, 4)*i);
	Eigen::VectorXd v0 = Eigen::VectorXd::Ones(4);
	v0[3] = 2+i;
	msg.SetValueVector(v0);
	return msg;
}

DataMsg msg2(int i) {
	DataMsg msg(10, STATE, SENSOR);
	msg.SetVarianceMatrix(Eigen::MatrixXd::Identity(2, 2)/i);
	Eigen::VectorXd v0 = Eigen::VectorXd::Ones(2);
	v0[3] = 2 + i;
	msg.SetValueVector(v0);
	return msg;
}

DataMsg msg3(int i) {
	DataMsg msg(15, STATE, SENSOR);
	msg.SetVarianceMatrix(Eigen::MatrixXd::Identity(4, 4)*i);
	Eigen::VectorXd v0 = Eigen::VectorXd::Ones(4);
	v0[3] = 2 + i;
	msg.SetValueVector(v0);
	return msg;
}


int main() {

	ZMQPublisher pub1("tcp://*:5555");
	ZMQPublisher pub2("tcp://*:5556");
	ZMQPublisher pub3("tcp://*:5557");


	int i = 0;
	while (true) {
		pub1.SendMsg(msg1(i));
		pub2.SendMsg(msg2(i));
		pub3.SendMsg(msg3(i));

		std::this_thread::sleep_for(std::chrono::duration<float, std::milli>(1000.f));
		i++;
	}
	return 0;

	/*
	while (true) {
		pub.SendString();
		std::this_thread::sleep_for(std::chrono::duration<float, std::milli>(100.f));
	}
	return 0;*/
}