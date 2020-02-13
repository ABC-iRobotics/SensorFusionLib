#include "Application.h"

using namespace SF;

void SF::Reciever::CallbackSamplingTimeOver(const Time& currentTime) {
	processorGuard.lock();
	if (processor)
		processor->CallbackSamplingTimeOver(currentTime);
	processorGuard.unlock();
}

void SF::Reciever::CallbackGotDataMsg(DataMsg & msg, const Time& currentTime) {
	processorGuard.lock();
	if (processor)
		processor->CallbackGotDataMsg(msg, currentTime);
	processorGuard.unlock();
}

void SF::Reciever::CallbackMsgQueueEmpty(const Time& currentTime) {
	processorGuard.lock();
	if (processor)
		processor->CallbackMsgQueueEmpty(currentTime);
	processorGuard.unlock();
}

SF::Reciever::Reciever()
	: processor(NULL), isRunning(false) {}

void SF::Reciever::Start(DTime Ts)
{
	if (!isRunning) {
		isRunning = true;
		toStop = false;
		t = std::thread([this, Ts]() {
			Run(Ts);
			isRunning = false;
		});
	}
}

void SF::Reciever::Stop(bool waitin)
{
	if (isRunning) {
		toStop = true;
		if (waitin)
			t.join();
	}
}

bool SF::Reciever::MustStop() const {
	return toStop;
}

void SF::Reciever::SetProcessor(Processor::ProcessorPtr processor_) {
	processorGuard.lock();
	processor = processor_;
	processorGuard.unlock();
}

void SF::Sender::SendDataMsg(const DataMsg & data) { data.print(); }

void SF::Sender::SendString(const std::string & str) { std::cout << str << std::endl; }

SF::Processor::Processor() : sender(NULL) {}

void SF::Processor::SetSender(Sender::SenderPtr sender_) {
	senderGuard.lock();
	sender = sender_;
	senderGuard.unlock();
}

void SF::Processor::CallbackSamplingTimeOver(const Time & currentTime) {
	senderGuard.lock();
	if (sender)
		sender->SendString("SamplingTimeOver");
	senderGuard.unlock();
}

void SF::Processor::CallbackGotDataMsg(DataMsg & msg, const Time & currentTime) {
	senderGuard.lock();
	if (sender)
		sender->SendDataMsg(msg);
	senderGuard.unlock();
}

void SF::Processor::CallbackMsgQueueEmpty(const Time & currentTime) {
	senderGuard.lock();
	if (sender)
		sender->SendString("MsgQueueEmpty");
	senderGuard.unlock();
}

Reciever::RecieverPtr SF::Application::GetRecieverPtr() const {
	return reciever;
}

Sender::SenderPtr SF::Application::GetSenderPtr() const {
	return sender;
}

Processor::ProcessorPtr SF::Application::GetProcessorPtr() const {
	return processor;
}

SF::Application::Application(Reciever::RecieverPtr reciever_,
	Processor::ProcessorPtr processor_, Sender::SenderPtr sender_) :
	reciever(reciever_), sender(sender_), processor(processor_) {
	if (reciever)
		reciever->SetProcessor(processor_);
	if (processor)
		processor->SetSender(sender_);
}

void SF::Application::Start(DTime Ts) {
	if (reciever)
		reciever->Start(Ts);
}

void SF::Application::Stop() {
	if (reciever)
		reciever->Stop();
}
