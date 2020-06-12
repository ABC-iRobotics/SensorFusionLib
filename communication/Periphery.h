#pragma once
#include "Forwarder.h"
#include "NetworkConfig.h"
namespace SF {

	/*! \brief Class to send datamsgs from sensors, basesystem, etc.
	*
	* The class is NOT thread safe! The same thread must initialize the class and call its methods.
	*/
	class Periphery : public Forwarder {
		using Forwarder::SetZMQOutput;

	public:
		Periphery(const std::string& address, int hwm = 10); /*!< Constructor, address e.g..: "tcp://*:5678" */

		Periphery(const NetworkConfig::ConnectionData& config); //!< Constructor

		void SendValue(unsigned char sensorID, const Eigen::VectorXd& value,
			DataType type, Time t = Now(), OperationType source = OperationType::SENSOR); /*!< Publish a DataMsg with a given values */

		void SendVariance(unsigned char sensorID, const Eigen::MatrixXd& variance,
			DataType type, Time t = Now(), OperationType source = OperationType::SENSOR); /*!< Publish a DataMsg with a given values */

		void SendValueAndVariance(unsigned char sensorID, const Eigen::VectorXd& value,
			const Eigen::MatrixXd& variance, DataType type,
			Time t = Now(), OperationType source = OperationType::SENSOR); /*!< Publish a DataMsg with a given values */
	};

}
