#include "Periphery.h"
#include "ClockSynchronizer.h"

using namespace SF;

int main() {
	try {
#ifdef UNIX
		NetworkConfig n;
		n.Add("networkconfig_1.json");
#else
		NetworkConfig n;
		n.Add("networkconfig_1_noipc.json");
#endif

	auto clockServer = InitClockSynchronizerServer(n.GetClockSyncData("remote1"));
	Periphery p1(n.GetPeripheryData("periphery1"));

	while (true)
		p1.SendValueAndVariance(8, Eigen::VectorXd::Ones(4) * 5, Eigen::MatrixXd::Identity(4, 4) * 7, OUTPUT);

	return 0;
	}
	catch (std::exception e) {
		std::cout << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
}