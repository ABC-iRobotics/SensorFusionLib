#pragma once
#include "Forwarder.h"
#include "FilterCore.h"
#include "SPDLogReader.h"

namespace SF {

	/* Log and zmqoutput can be set via Forwarder::SetLogger, Forwarder::SetZMQOutput
	*
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
		RealTimeSimulator(const char* filename, FilterCore::FilterCorePtr filterCorePtr);

		void Start(DTime Ts);

		void Stop(bool waitin = true);

		bool MustStop() const;

		bool IsRunning() const;

		~RealTimeSimulator();
	};
}
