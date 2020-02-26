#include "AppLayer.h"

using namespace SF;

void SF::AppLayer::CallbackSamplingTimeOver(const Time & currentTime) {
	ForwardSamplingTimeOver(currentTime);
}

void SF::AppLayer::CallbackGotDataMsg(const DataMsg & msg, const Time & currentTime) {
	ForwardDataMsg(msg, currentTime);
}

void SF::AppLayer::CallbackGotString(const std::string & msg, const Time & currentTime) {
	ForwardString(msg, currentTime);
}

void SF::AppLayer::CallbackMsgQueueEmpty(const Time & currentTime) {
	ForwardMsgQueueEmpty(currentTime);
}

void SF::AppLayer::AddNextLayer(AppLayerPtr ptr) {
	nextLayerGuard.lock();
	nextLayers.push_back(ptr);
	nextLayerGuard.unlock();
}

void SF::AppLayer::ForwardMsgQueueEmpty(const Time & currentTime) {
	nextLayerGuard.lock();
	for (auto it : nextLayers)
		it->CallbackMsgQueueEmpty(currentTime);
	nextLayerGuard.unlock();
}

void SF::AppLayer::ForwardSamplingTimeOver(const Time & currentTime) {
	nextLayerGuard.lock();
	for (auto it : nextLayers)
		it->CallbackSamplingTimeOver(currentTime);
	nextLayerGuard.unlock();
}

void SF::AppLayer::ForwardDataMsg(const DataMsg & msg, const Time & currentTime) {
	nextLayerGuard.lock();
	for (auto it : nextLayers)
		it->CallbackGotDataMsg(msg, currentTime);
	nextLayerGuard.unlock();
}

void SF::AppLayer::ForwardString(const std::string & msg, const Time & currentTime) {
	nextLayerGuard.lock();
	for (auto it : nextLayers)
		it->CallbackGotString(msg, currentTime);
	nextLayerGuard.unlock();
}
