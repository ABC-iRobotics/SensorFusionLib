#pragma once
#include "Forwarder.h"
#include "FilterCore.h"
#include "SPDLogReader.h"

namespace SF {

	/* Log and zmqoutput can be set via Forwarder::SetLogger, Forwarder::SetZMQOutput
	*
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
		SteppableSimulator(const char* filename, FilterCore::FilterCorePtr filterCore_);

		void Start(DTime Ts);

		void Stop(bool waitin = true);

		bool MustStop() const;

		bool IsRunning() const;

		~SteppableSimulator();
	};
}
