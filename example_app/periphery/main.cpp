#include "NetworkConfig.h"
#include "Periphery.h"
#include "ClockSynchronizer.h"
#include <iostream>

using namespace SF;

int main() {
#ifdef UNIX
	NetworkConfig n("networkconfig_1.json");
#else
	NetworkConfig n("networkconfig_1_noipc.json");
#endif

	auto clockServer = InitClockSynchronizerServer(n.GetClockSyncData("remote1"));
	Periphery p1(n.GetPeripheryData("periphery1"));

	while (true)
		p1.SendValueAndVariance(8, Eigen::VectorXd::Ones(4) * 5, Eigen::MatrixXd::Identity(4, 4) * 7, OUTPUT);


	return 0;
	
}