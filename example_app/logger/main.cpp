#include "Logger.h"
#include "ClockSynchronizer.h"
#include "NetworkConfig.h"
#include "Periphery.h"

#include <iostream>

using namespace SF;

int main() {
	NetworkConfig n("networkconfig_1.json");

	std::string filename = "log_output" + std::to_string(Now().time_since_epoch().count()) + ".log";

	Logger l(filename);
	l.AddPeripheries(n);
	l.Start(DTime(2000));

	Periphery p1(n.GetPeripheryData("localperiphery1"));
#ifdef UNIX
	Periphery p2(n.GetPeripheryData("localperiphery2"));
#endif
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	for (int i = 0; i < 10000; i++) {
		p1.SendValueAndVariance(5, Eigen::VectorXd::Ones(4) * i, Eigen::MatrixXd::Identity(4, 4) * 7, OUTPUT);
#ifdef UNIX
		p2.SendValue(10, Eigen::VectorXd::Ones(7) * i, OUTPUT);
#endif
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	return 0;
	
}