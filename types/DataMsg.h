#pragma once

#include "defs.h"
#include "StatisticValue.h"
#include <memory>

namespace SF {

	/*! \brief General message format containing a StatisticValue, a timestamp and information about its DataType and source
	*
	* Its used in Lib_SensorFusion, Lib_Communication and Lib_Plotter as well
	*/
	class DataMsg {
		DataType dataType;
		OperationType dataSource;
		unsigned char sourceID;
		Eigen::VectorXd value;
		bool hasValue;
		Eigen::MatrixXd variance;
		bool hasVariance;
		Time time;

	public:
		Time GetTime() const; /*!< Timestamp getter */

		DataMsg(); /*!< Empty constructor */

		DataMsg(unsigned char ID, DataType type, OperationType source, const Time& time_ = Now()); /*!< Constructor without specifying value and variance */

		DataMsg(unsigned char ID, DataType type, OperationType source,
			StatisticValue data, const Time& time = Now()); /*!< Constructor with StatisticValue */

		bool IsInvalid() const; /*!< To check if the instance is empty */

		bool HasValue() const; /*!< To check if value vector was set */

		bool HasVariance() const; /*!< To check if covariance matrix was set */

		void ClearValue();  /*!< To remove the stored value vector */

		void ClearVariance();  /*!< To remove the stored variance vector */

		Eigen::VectorXd GetValue() const;  /*!< To get the stored value vector */

		Eigen::MatrixXd GetVariance() const; /*!< To get the stored variance vector */

		unsigned char GetSourceID() const; /*!< To get source ID */

		DataType GetDataType() const; /*!< To get DataType */

		OperationType GetDataSourceType() const; /*!< To get the type of the source as an OperationType */

		void print() const; /*!< To show its content */

		void SetVarianceMatrix(const Eigen::MatrixXd& m); /*!< To set covariance matrix */

		void SetValueVector(const Eigen::VectorXd& v); /*!< To set the value vector */

		typedef std::shared_ptr<DataMsg> DataMsgPtr; /*!< std::shared_ptr for class DataMsg */

		typedef std::vector<DataMsgPtr> DataMsgPtrList; /*!< std::vector for instances of std::shared_ptr<DataMsg> */

		template<class... Args>
		static DataMsg::DataMsgPtr CreateSharedPtr(Args&&... args) {
			return std::make_shared<DataMsg>(args...);
		} /*!< Init std::shrared_ptr<DataMsg> */

		void ApplyOffset(DTime offset); /*!< Modify timestamp as timestamp += offset*/

		bool operator!=(const DataMsg& data) const;

		bool operator==(const DataMsg& data) const;
	};

}
