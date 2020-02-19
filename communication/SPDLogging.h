#pragma once
#include "spdlog/logger.h"
#include<fstream>
#include "Application.h"

namespace SF {

	/*! \brief Class for reading spd log
	*
	*/
	class SPDLogReader {
	public:
		SPDLogReader(std::string filename); /*!< Constructor */

		MsgType readNextRow(); /*!< Read the next row, returns its type */

		MsgType getLatestRowType() const;  /*!< Get the type of the latest row read */

		Time getLatestTimeStamp() const; /*!< Get the timestamp of the latest row read */

		const std::string& getLatestRowIf() const; /*!< Get the std::string read last time */

		const DataMsg& getLatestDataMsgIf() const; /*!< Get the DataMsg read last time */

		typedef std::shared_ptr<SPDLogReader> LogReaderPtr; /*!< std::shared_ptr for ILogReader */
	private:
		std::ifstream stream;
		Time firstTime;
		Time latestTime;
		DataMsg latestMsg;
		MsgType latestRowType;
		std::string latestRow;
		static const int BUFSIZE = 200;
		char buf[BUFSIZE];
	};

	/*! \brief Sender implementation for reading from spd log
	*
	*/
	class SPDSender : public Sender {
		std::shared_ptr<spdlog::logger> my_logger;

		spdlog::memory_buf_t buf;

	public:
		SPDSender(std::string filename, std::string loggername); /*!< Constructor */

		~SPDSender();  /*!< Destructor */

		SPDSender(const SPDSender&) = delete;

		void SendDataMsg(const DataMsg& data) override;

		void SendString(const std::string& str) override;
	};

	/*! \brief Class to emulate the recieving (filtering) situation from a log file
	*
	* Steppable simulation: processing time, Visual Studio debug does not influence its behavior
	*
	* \htmlonly
	* <embed src="Reading log - steppable.pdf" width="700px" height="870px" href="Reading log - steppable.pdf"></embed>
	* \endhtmlonly
	*
	* \htmlonly
		* <embed src="Reading log - real time.pdf" width="700px" height="950px" href="Reading log - real time.pdf"></embed>
		* \endhtmlonly
	*/
	class SPDReciever : public Reciever {
		SPDLogReader logread;
		bool realtime;

		void AddPeriphery(const PeripheryProperties& prop) override {};

	public:
		SPDReciever(const std::string& filename, bool realTime_) : logread(filename), realtime(realTime_) {}

		void _Run(DTime Ts) override;

		/*! The core of the thread that reads the log and forwards it to the processor with the logged time stamps without checking the real time clocks
		*
		* \htmlonly
		* <embed src="Reading log - steppable.pdf" width="700px" height="870px" href="Reading log - steppable.pdf"></embed>
		* \endhtmlonly
		*/
		void _RunSteppable(DTime Ts, DTime recieveTime = DTime(20));

		/*! The core of the thread that reads the log and forwards it to the processor according to the real time clocks
		*
		* \htmlonly
		* <embed src="Reading log - real time.pdf" width="700px" height="950px" href="Reading log - real time.pdf"></embed>
		* \endhtmlonly
		*/
		void _RunRT(DTime Ts) {
			DTime tRead(15);
			Time tNext;
			//DTime offset;
			std::function<Time()> Now2;
			bool got, first = true;
			while (true) {
				// Read the next row
				logread.readNextRow();
				// Initialize the variables
				if (first) {
					DTime offset = duration_cast(Now() - logread.getLatestTimeStamp());
					Now2 = [offset]() { return Now() - offset; };
					tNext = logread.getLatestTimeStamp() + Ts;
					first = false;
					got = false;
				}
				// Inner iteration
				while (true) {
					// exit condition
					if (MustStop() || logread.getLatestRowType() == NOTHING)
						return;
					// Sampling ended
					if ( (Now2() > tNext) || ( logread.getLatestTimeStamp() > tNext && !got ) ) {
						if (Now2() < tNext)
							std::this_thread::sleep_for(tNext - Now2());
						CallbackSamplingTimeOver(Now2());
						tNext += Ts;
						got = false;
						continue;
					}
					// Read queue is empty
					if (got && (logread.getLatestTimeStamp() > Now2() + tRead)) {
						CallbackMsgQueueEmpty(Now2());
						got = false;
						continue;
					}
					break;
				}
				if (Now2() < logread.getLatestTimeStamp())
					std::this_thread::sleep_for(logread.getLatestTimeStamp() - Now2());
				// Process the current row
				switch (logread.getLatestRowType()) {
				case DATAMSG:
					CallbackGotDataMsg(logread.getLatestDataMsgIf(), Now2());
					got = true;
					break;
				case TEXT:
					CallbackGotString(logread.getLatestRowIf(), Now2());
					break;
				}
			}










			bool first = true;
			DTime offset;
			Time start;
			long iIteration = 1;
			while (!MustStop()) {
				if (logread.readNextRow() == NOTHING)
					return;
				if (first) {
					start = Now();
					offset = duration_cast(start - logread.getLatestTimeStamp());
					first = false;
				}
				if (start + iIteration * Ts < Now()) {

				}
				while (Now() > logread.getLatestTimeStamp() + offset)
					std::this_thread::sleep_for(std::chrono::microseconds(10));
				switch (logread.getLatestRowType()) {
				case DATAMSG:
					
					/*
					if (!first && lastReadDataMsgTime + recieveTime < logread.getLatestTimeStamp()) {
						CallbackMsgQueueEmpty();
						if (logread.getLatestTimeStamp() > start + iIteration * Ts) {
							CallbackSamplingTimeOver();
							iIteration++;
						}
					}
					CallbackGotDataMsg(logread.getLatestDataMsgIf(), logread.getLatestTimeStamp());
					lastReadDataMsgTime = logread.getLatestTimeStamp();
					if (first) {
						start = lastReadDataMsgTime;
						first = false;
					}
					*/
					break;
				case TEXT:
					CallbackGotString(logread.getLatestRowIf(), logread.getLatestTimeStamp());
					break;
				default:
					return;
				}

			}






			auto type = logread.readNextRow();
			if (type == NOTHING)
				return;
			Time recieved = logread.getLatestTimeStamp();
			if (realtime) {
				while (recieved < Now())
					std::this_thread::sleep_for(std::chrono::microseconds(1));
			}
			switch (logread.readNextRow())
			{
			case DATAMSG:
			case TEXT:

			default:
				break;
			}

		}

	};

}