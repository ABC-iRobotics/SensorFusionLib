#include "Periphery.h"

using namespace SF;

int main(int argc, char *argv[]) {
	if (argc != 2) {
		printf("Expected command line inputs: network_config_json_filename, periphery_name\n");
		return 0;
	}
	
	try {
		NetworkConfig n;
		n.Add(argv[0]);

		Periphery p1(n.GetPeripheryData(argv[1]));

		while (true)
			p1.SendValueAndVariance(8, Eigen::VectorXd::Ones(4) * 5, Eigen::MatrixXd::Identity(4, 4) * 7, OUTPUT);

		return 0;
	}
	catch (std::exception e) {
		std::cout << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
}