#include "msg2buf.h"
#include <thread>
#include "ZMQPublisher.h"

int main() {
	
	ZMQPublisher pub;

	DataMsg msg(5, DataMsg::MEASUREMENT, getTimeInMicroseconds());
	msg.SetVarianceMatrix(Eigen::MatrixXd::Identity(4, 4));
	Eigen::VectorXd v0 = Eigen::VectorXd::Ones(4);
	v0[3] = 2;
	msg.SetValueVector(v0);
	msg.print();


	int i = 0;
	while (true) {
		msg.timestamp_in_us = getTimeInMicroseconds();
		msg.sourceID = i;
		pub.SendMsg(msg);
		msg.print();
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