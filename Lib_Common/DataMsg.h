#pragma once

#include "defs.h"
#include "TimeMicroSec.h"
#include "StatisticValue.h"

/*! \brief General message format containing a StatisticValue, a timestamp and information about its DataType and source
*
* Its used in Lib_SensorFusion, Lib_Communication and Lib_Plotter as well
*/
class DataMsg {
	DataType dataType;
	OperationType dataSource;
	bool empty;
	unsigned char sourceID;
	Eigen::VectorXd value;
	bool hasValue;
	Eigen::MatrixXd variance;
	bool hasVariance;
	TimeMicroSec time;

public:
	DataMsg(); /*!< Empty constructor */

	DataMsg(unsigned char ID, DataType type, OperationType source,
		TimeMicroSec time_ = TimeMicroSec()); /*!< Constructor where the data can be set later*/

	DataMsg(unsigned char ID, DataType type, OperationType source,
		StatisticValue data, TimeMicroSec time_ = TimeMicroSec()); /*!< Constructor */

	bool IsEmpty() const; /*!< To check if the instance is empty */

	bool HasValue() const; /*!< To check if value vector was set */

	bool HasVariance() const; /*!< To check if covariance matrix was set */

	void ClearValue();  /*!< To remove the stored value vector */

	void ClearVariance();  /*!< To remove the stored variance vector */

	Eigen::VectorXd GetValue() const;  /*!< To get the stored value vector */

	Eigen::MatrixXd GetVariance() const; /*!< To get the stored variance vector */

	unsigned char GetSourceID() const; /*!< To get source ID */

	DataType GetDataType() const; /*!< To get DataType */

	OperationType GetDataSourceType() const; /*!< To get the type of the source as an OperationType */

	TimeMicroSec GetTime() const;  /*!< To get its timestamp */

	void print() const; /*!< To show its content */

	void SetVarianceMatrix(const Eigen::MatrixXd& m); /*!< To set covariance matrix */

	void SetValueVector(const Eigen::VectorXd& v); /*!< To set the value vector */
};

