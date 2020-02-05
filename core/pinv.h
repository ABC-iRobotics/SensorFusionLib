#pragma once

#include <Eigen/QR> 

namespace SF {

	Eigen::MatrixXd pinv(const Eigen::MatrixXd& in);

	bool eq(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b); // Eigen::MatrixXd::isApprox???

}