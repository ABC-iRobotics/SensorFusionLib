#include "Common/unity.h"
void setUp() {}
void tearDown() {}

#include "ClockSynchronizer.h"
#include "zmqPublisher.h"
#include "zmqSubscriber.h"
#include <iostream>

using namespace SF;

void testLocalClockSynchronizationOnTCP(int nCases = 5, int nmsgs = 10000) {
	ClockSynchronizerServer sv("tcp://*:5555");
	sv.StartServer();
	for (int i = 0; i < nCases; i++) {
		long long offset = GetOffsetFromServerTime("tcp://localhost:5555", nmsgs);
		std::cout << "Computed offset: " << offset << "us\n";
		TEST_ASSERT_LESS_THAN(offset, -3);
		TEST_ASSERT_GREATER_THAN(offset, 3);
	}
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() {testLocalClockSynchronizationOnTCP(); });
	//TODO: IPC
	return UNITY_END();
}
