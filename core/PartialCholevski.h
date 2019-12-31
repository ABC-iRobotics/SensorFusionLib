#pragma once

#include "Eigen/Dense"


/* \brief Cholevski-factorization of positive semi-definite matrices in the given columns
 *
 * Based on the Cholevski-Crout algrithm
 */
Eigen::MatrixXd PartialChol(Eigen::MatrixXd a, Eigen::VectorXi v);