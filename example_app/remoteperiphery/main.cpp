#include "Periphery.h"
#include "ClockSynchronizer.h"
#include "PrintNestedException.h"

using namespace SF;

int main(int argc, char *argv[]) {
	if (argc != 4) {
		printf("%d Expected command line inputs: network_config_json_filename, remote_name, periphery_name\n", argc);
		return 0;
	}

	try {
		NetworkConfig n;
		n.Add(argv[1]);

		auto clockServer = InitClockSynchronizerServer(n.GetClockSyncData(argv[2]));
		Periphery p1(n.GetPeripheryData(argv[3]));

		while (true)
			p1.SendValueAndVariance(10, Eigen::VectorXd::Ones(4) * 5, Eigen::MatrixXd::Identity(4, 4) * 7, OUTPUT);

		return 0;
	}
	catch (const std::exception& e) {
		print_exception(e);
		exit(EXIT_FAILURE);
	}
}