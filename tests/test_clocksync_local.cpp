#include "common/unity.h"
void setUp() {}
void tearDown() {}

#include "ClockSynchronizer.h"
#include <thread>
#include <iostream>

using namespace SF;

void testLocalClockSynchronizationOnTCP(int nCases = 5, int nmsgs = 10000) {
	auto c1 = InitClockSynchronizerServer("tcp://*:5565");
	

	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery("localhost","5565");
	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery("localhost","5565");
	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery("localhost","5566");
	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery("localhost","5567");
	
	while (GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress("localhost"))
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	
	GetPeripheryClockSynchronizerPtr()->PrintStatus();

	TEST_ASSERT_LESS_THAN(4, abs(GetPeripheryClockSynchronizerPtr()->GetOffset("localhost").count()));
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() {testLocalClockSynchronizationOnTCP(); });
	//TODO: IPC
	// TODO: add non-local tests - skipped locally
	return UNITY_END();
}
