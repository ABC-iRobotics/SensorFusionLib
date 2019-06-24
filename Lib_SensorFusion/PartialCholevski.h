#pragma once

#include "Eigen/Dense"

// The Cholevski-Craut algorithm with some midifications
Eigen::MatrixXd PartialChol(Eigen::MatrixXd a, Eigen::VectorXi v);