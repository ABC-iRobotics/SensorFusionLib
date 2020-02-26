#pragma once

#include<vector>
#include<mutex>
#include"DataMsg.h"

namespace SF {

	/*
	*
	*	Callbacks: called by previous layers
	*
	*   Forwards: call the next layers
	*
	*/
	class AppLayer {
	public:
		typedef std::shared_ptr<AppLayer> AppLayerPtr;

		virtual void CallbackSamplingTimeOver(const Time& currentTime = Now()); /*!< Callback called in each sampling time - input: time */

		virtual void CallbackGotDataMsg(const DataMsg& msg, const Time& currentTime = Now()); /*!< Callback called if new DataMsg recieved */

		virtual void CallbackGotString(const std::string& msg, const Time& currentTime = Now()); /*!< Callback called if new string recieved */

		virtual void CallbackMsgQueueEmpty(const Time& currentTime = Now()); /*!< Callback called if the DataMsgs in the queue were read */

		void AddNextLayer(AppLayerPtr ptr);

	protected:
		void ForwardMsgQueueEmpty(const Time& currentTime = Now());

		void ForwardSamplingTimeOver(const Time& currentTime = Now());

		void ForwardDataMsg(const DataMsg& msg, const Time& currentTime = Now());

		void ForwardString(const std::string& msg, const Time& currentTime = Now());

	private:
		std::vector<AppLayerPtr> nextLayers;

		std::mutex nextLayerGuard;
	};
}
