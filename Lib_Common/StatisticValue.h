#pragma once

#include "Eigen/Dense"
#include <iostream>


/*! \brief Class to represent statistic variables as a vector and a covariance matrix
*
*/
struct StatisticValue {
	Eigen::VectorXd vector; /*!< The vector containing the value */

	Eigen::MatrixXd variance; /*!< Covariance matrix  */

	bool isIndependent; /*!< Describes if the varince matrix is diagonal, then the variables are independent */

	StatisticValue(const Eigen::VectorXd& vector_, const Eigen::MatrixXd& variance_,
		bool isIndependent_ = false); /*!< Simple constructor */

	StatisticValue(const Eigen::VectorXd& vector_);  /*!< Constructor, that sets zero covariance matrix */

	StatisticValue(size_t n); /*!< Constructor, that sets zero values and covariance matrix */

	StatisticValue(); /*!< Constructor, that sets zero size value and covariance matrix */

	Eigen::Index Length() const; /*!< Returns the number of variables */

	void Insert(Eigen::Index StartIndex, const StatisticValue& value); /*!< Insert a block from the StartIndex, setting zero cross variances with the other values */

	StatisticValue GetPart(Eigen::Index StartIndex, Eigen::Index Length) const; /*!< Returns a block*/

	void Add(const StatisticValue& value); /*!< Concatenate statistic variables setting zero cross variances */
};

std::ostream &operator<<(std::ostream &os, const StatisticValue &m);
