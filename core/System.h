#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include "defs.h"
#include "StatisticValue.h"
#include <string>

namespace SF {

	/*! \brief VAR_STATE: STATE, VAR_EXTERNAL: DISTURBANCE or NOISE depending on the context
	*
	*
	*/
	enum VariableType { VAR_STATE, VAR_EXTERNAL };

	/*! \brief Abstract superclass for BaseSystem and Sensor classes providing general interface for their common properties
	*
	*
	*/
	class System {
		unsigned int ID; /*!< User defined ID */
	public:
		unsigned int getID() const { return ID; } /*!< Get user defined ID */

		virtual unsigned int getNumOfStates() const = 0; /*!< Get number of state variables */

		virtual unsigned int getNumOfDisturbances() const = 0; /*!< Get number of disturbance values */

		virtual unsigned int getNumOfOutputs() const = 0; /*!< Get number of outputs */

		virtual unsigned int getNumOfNoises() const = 0; /*!< Get number of noises */

		virtual std::vector<std::string> getStateNames() const; /*!< Get names of state variables */

		virtual std::vector<std::string> getNoiseNames() const;  /*!< Get names of disturbance values */

		virtual std::vector<std::string> getDisturbanceNames() const; /*!< Get names of outputs */

		virtual std::vector<std::string> getOutputNames() const; /*!< Get names of noises */

		virtual std::string getName() const; /*!< Get name of the system */

		virtual const Eigen::VectorXi& getIfStateIsRad() const;

		virtual const Eigen::VectorXi& getIfOutputIsRad() const;

	protected:
		System(unsigned int ID); /*!< Constructor */

		~System();  /*!< Destructor */

	public:
		unsigned int getNumOf(DataType type) const;  /*!< General interface to get the length of STATE, DISTURBANCE, NOISE or OUTPUT vector */

		std::vector<std::string> getNames(DataType type) const; /*!< General interface to get the names of elements of STATE, DISTURBANCE, NOISE or OUTPUT vector */

		typedef std::shared_ptr<System> SystemPtr; /*!< Shared pointer type for the system class */

		static DataType getInputValueType(TimeUpdateType outType, VariableType inType); /*!< The external input of STATE_UPDATE is DISTURBANCE, and OUTPUT_UPDATE's is NOISE */

		static DataType getOutputValueType(TimeUpdateType outType); /*!< The result of STATE_UPDATE is STATE, and OUTPUT_UPDATE's is OUTPUT */

	protected:
		void _systemTest() const; /*!< To check the consistency of the defined functions */
	};

}
