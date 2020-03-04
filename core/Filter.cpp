#include "Filter.h"

using namespace SF;

void SF::Filter::CallbackSamplingTimeOver(const Time & t) {
	Step(duration_cast(t - lastTimeStep));
	lastTimeStep = t;
}

void SF::Filter::CallbackMsgQueueEmpty(const Time & t) {
	Step(duration_cast(t - lastTimeStep));
	lastTimeStep = t;
}
