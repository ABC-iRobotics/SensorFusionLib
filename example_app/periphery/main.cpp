#include "Periphery.h"
#include "PrintNestedException.h"

using namespace SF;

int main(int argc, char *argv[]) {
	if (argc != 3) {
		printf("Expected command line inputs: network_config_json_filename, periphery_name\n");
		return 0;
	}
	
	try {
		NetworkConfig n;
		n.Add(argv[1]);

		Periphery p1(n.GetPeripheryData(argv[2]));

		while (true)
			p1.SendValueAndVariance(8, Eigen::VectorXd::Ones(4) * 5, Eigen::MatrixXd::Identity(4, 4) * 7, OUTPUT);

		return 0;
	}
	catch (const std::exception& e) {
		print_exception(e);
		exit(EXIT_FAILURE);
	}
}