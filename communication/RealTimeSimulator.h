#pragma once
#include "Forwarder.h"
#include "FilterCore.h"
#include "SPDLogReader.h"

#include <thread>

namespace SF {

	/*! \brief Class for testing filtering methods real-time on logged data
	*
	* The filtering is done via a FilterCore instance set in the constructor
	*
	* The results are forwarded via inherited methods of class Forwarder (see Forwarder::SetLogger, Forwarder::SetZMQOutput)
	*/
	class RealTimeSimulator : public Forwarder {
		SPDLogReader logread; // Reads a log

		FilterCore::FilterCorePtr filterCore;

		bool toStop;

		bool isRunning;

		std::thread t;

		using Forwarder::ForwardDataMsg;

		using Forwarder::ForwardString;

		void _run(DTime Ts);

	public:
		RealTimeSimulator(const std::string& filename, FilterCore::FilterCorePtr filterCorePtr); //!< Constructor

		void Start(DTime Ts); //!< Start recieving thread that will call filtercore and forwarding methods

		void Stop(bool waitin = true); //!< Stop reading thread, called filtercore and forwarding methods

		bool MustStop() const; //!< To check if stop was called

		bool IsRunning() const; //!< To check if reading thread is running

		~RealTimeSimulator(); //!<  Destructor
	};
}
