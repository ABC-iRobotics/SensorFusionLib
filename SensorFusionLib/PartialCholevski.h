#pragma once

#include "Eigen/Dense"

Eigen::MatrixXd PartialChol(Eigen::MatrixXd a, Eigen::VectorXi v);

void test();