#include "common/unity.h"
void setUp() {}
void tearDown() {}

#include "Periphery.h"
#include"Logger.h"
#include"SPDLogging.h"

using namespace SF;

void localtest() {
	std::string filename = "local_test_" + std::to_string(Now().time_since_epoch().count()) + ".log";
	{
		NetworkConfig n("networkconfig_local.json");

		Logger l(filename);
		l.AddPeripheries(n);
		l.Start(DTime(2000));

		Periphery p1(n.GetPeripheryData("sensor1"));
		Periphery p2(n.GetPeripheryData("sensor2"));

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		for (int i = 0; i < 10000; i++) {
			p1.SendValueAndVariance(5, Eigen::VectorXd::Ones(4) * i, Eigen::MatrixXd::Identity(4, 4) * 7, OUTPUT);
			p2.SendValue(10, Eigen::VectorXd::Ones(7) * i, OUTPUT);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	}
	int gotfrom5 = 0;
	int gotfrom10 = 0;
	{
		SPDLogReader reader(filename);
		while (reader.readNextRow() != MsgType::NOTHING) {
			switch (reader.getLatestRowType()) {
			case MsgType::DATAMSG:
				switch (reader.getLatestDataMsgIf().GetSourceID()) {
				case 5:
					gotfrom5++;
					break;
				case 10:
					gotfrom10++;
					break;
				}
				break;
			case MsgType::TEXT:
				// ?
				break;
			}
		}
	}
	// delete created log file
	if (remove(filename.c_str()) != 0)
		perror("Error deleting file");
	TEST_ASSERT(gotfrom5 > 0);
	TEST_ASSERT(gotfrom10 > 0);
	std::cout << "Msgs got (max 10000): " << gotfrom5 << " " << gotfrom10 << std::endl;
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST(localtest);
	return UNITY_END();
}
