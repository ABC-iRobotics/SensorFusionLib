#include "Common/unity.h"
void setUp() {}
void tearDown() {}

#include "ZMQClockSynchronizer.h"
#include "zmqPublisher.h"
#include "zmqSubscriber.h"
#include <iostream>

using namespace SF;

void testLocalClockSynchronizationOnTCP(int nCases = 5, int nmsgs = 10000) {
	ZMQClockSynchronizerServer sv("tcp://*:5555");
	sv.StartServer();
	for (int i = 0; i < nCases; i++) {
		ClockSynchronizerClient client;
		client.UpdateOffsetFromServerTime("tcp://localhost:5555", nmsgs);
		long long offset = client.GetOffset().count();
		std::cout << "Computed offset: " << offset << " ns\n";
		TEST_ASSERT_LESS_THAN(offset, -4000);
		TEST_ASSERT_GREATER_THAN(offset, 4000);
	}
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() {testLocalClockSynchronizationOnTCP(); });
	//TODO: IPC
	// TODO: add non-local tests - skipped locally
	return UNITY_END();
}
