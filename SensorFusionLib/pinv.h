#pragma once

#include <Eigen/QR> 

Eigen::MatrixXd pinv(const Eigen::MatrixXd& in);

bool eq(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b);