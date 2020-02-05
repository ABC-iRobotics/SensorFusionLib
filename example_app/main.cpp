#include "pinv.h"

#include <iostream>

using namespace SF;

int main() {
	
	Eigen::MatrixXd M = Eigen::MatrixXd::Identity(4, 4);

	std::cout << pinv(M) << std::endl;

	return 0;
	
}