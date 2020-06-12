#pragma once
#include "Forwarder.h"
#include "FilterCore.h"
#include "SPDLogReader.h"

namespace SF {

	/*! \brief Class for testing filtering methods not real-time on logged data for debug
	*
	* The filtering is done via a FilterCore instance set in the constructor
	*
	* The results are forwarded via inherited methods of class Forwarder (see Forwarder::SetLogger, Forwarder::SetZMQOutput)
	*/
	class SteppableSimulator : public Forwarder {
		SPDLogReader logread; // Reads a log

		FilterCore::FilterCorePtr filterCore;

		void _run(DTime Ts);

		bool toStop;

		bool isRunning;

		std::thread t;

		using Forwarder::ForwardDataMsg;

		using Forwarder::ForwardString;

	public:
		SteppableSimulator(const std::string& logfilename, FilterCore::FilterCorePtr filterCore_); //!< Constructor

		void Start(DTime Ts); //!< Start recieving thread that will call filtercore and forwarding methods

		void Stop(bool waitin = true); //!< Stop reading thread, called filtercore and forwarding methods

		bool MustStop() const; //!< To check if stop was called

		bool IsRunning() const; //!< To check if reading thread is running

		~SteppableSimulator(); //!<  Destructor
	};
}
