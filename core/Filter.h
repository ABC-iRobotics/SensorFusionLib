#pragma once

#include "AppLayer.h"

namespace SF {

	/*! \brief
	*/
	class Filter : public AppLayer {
		Time lastTimeStep;

	public:
		void CallbackSamplingTimeOver(const Time& t) override;

		/*! \brief Virtual function for filtering function including time update & measurement update
		*
		* It must call SystemManager::StepClock, SystemManager::PredictionDone, SystemManager::FilteringDone protected functions
		*/
		virtual void Step(DTime dT) = 0;

		void CallbackMsgQueueEmpty(const Time& t) override;
	};

}