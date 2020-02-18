#include "Common/unity.h"
void setUp() {}
void tearDown() {}

#include "IClockSynchronizer.h"
#include <iostream>

using namespace SF;

void testLocalClockSynchronizationOnTCP(int nCases = 5, int nmsgs = 10000) {
	auto c1 = InitClockSynchronizerServer("tcp://*:5565");
	auto c2 = InitClockSynchronizerServer("tcp://*:5566");
	auto c3 = InitClockSynchronizerServer("tcp://*:5567");
	

	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery(1,"tcp://localhost:5565");
	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery(10, "tcp://localhost:5565");
	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery(2,"tcp://localhost:5566");
	GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery(3,"tcp://localhost:5567");
	
	while (GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress(1) ||
		GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress(2) ||
		GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress(3) ||
		GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress(10))
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

	TEST_ASSERT_LESS_THAN(4, abs(GetPeripheryClockSynchronizerPtr()->GetOffset(1).count()));
	TEST_ASSERT_LESS_THAN(4, abs(GetPeripheryClockSynchronizerPtr()->GetOffset(2).count()));
	TEST_ASSERT_LESS_THAN(4, abs(GetPeripheryClockSynchronizerPtr()->GetOffset(3).count()));
	TEST_ASSERT_LESS_THAN(4, abs(GetPeripheryClockSynchronizerPtr()->GetOffset(10).count()));
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() {testLocalClockSynchronizationOnTCP(); });
	//TODO: IPC
	// TODO: add non-local tests - skipped locally
	return UNITY_END();
}
