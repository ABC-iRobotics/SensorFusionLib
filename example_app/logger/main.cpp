#include "Logger.h"
#include "NetworkConfig.h"
#include "PrintNestedException.h"
#include <iostream>

using namespace SF;

int main(int argc, char *argv[]) {
	if (argc != 2) {
		printf("Expected command line input: network_config_json_filename\n");
		return 0;
	}

	try {
		NetworkConfig n;
		n.Add(argv[1]);

		std::string filename = "log_output_" + std::to_string(Now().time_since_epoch().count()) + ".log";
		Logger l(filename);
		l.AddPeripheries(n);
		l.Start(DTime(2000));

		while (true)
			;
	}
	catch (const std::exception& e) {
		print_exception(e);
		exit(EXIT_FAILURE);
	}
}