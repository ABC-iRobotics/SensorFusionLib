#include "common/unity.h"
void setUp() {}
void tearDown() {}

#include <thread>
#include <iostream>

#include"Periphery.h"
#include"Logger.h"
#include"CentralUnit.h"
#include"SPDLogging.h"

using namespace SF;

template<class T>
class Statistics {
	int N = 0;
	T sum = T(0);
	T max = T(0);
	static const int K = 5;
	T buf[K];
	int i = 0;
	int filled = 0;
	T maxK = T(0);
public:
	void Add(T value) {
		N++;
		sum += value;
		max = max > value ? max : value;

		buf[i % K] = value;
		i++;
		if (filled < K)
			filled++;
		else {
			T sumK = T(0);
			for (int k = 0; k < K; k++)
				sumK += buf[k];
			sumK /= K;
			maxK = maxK > sumK ? maxK : sumK;
		}
	}

	T GetMax() const { return max; }

	T GetMaxK() const { return maxK; }

	T GetAverage() const { if (N > 0) return sum / N; else return T(0); }
};

class Tester : public AppLayer {
	Statistics<DTime> between5msgs;
	Statistics<DTime> afterfifthmsg;
	Statistics<DTime> samplingTime;
	int j = 0;
	Time start;
	bool last = false;
	int errors = 0;
	int wrongqueueended = 0;
	bool firstsamplingtimeover = true;
	Time lastSamplingTime;
	Time lastTime;
public:

	void CallbackSamplingTimeOver(const Time& currentTime = Now()) override {
		//printf(" SamplingTimeOver %lld\n", duration_cast(currentTime.time_since_epoch()).count());
		if (!firstsamplingtimeover)
			samplingTime.Add(duration_cast(currentTime - lastSamplingTime));
		else
			firstsamplingtimeover = false;
		lastSamplingTime = currentTime;
		if (last) {
			afterfifthmsg.Add(duration_cast(currentTime - lastTime));
			last = false;
		}
		lastTime = currentTime;
	}

	void CallbackGotDataMsg(const DataMsg& msg, const Time& currentTime = Now()) override {
		//printf(" DATAMSG %lld\n", duration_cast(currentTime.time_since_epoch()).count());
		if (last) {
			errors++;
			last = false;
		}
		if (j % 5 == 0)
			start = currentTime;
		j++;
		if (j % 5 == 0) {
			between5msgs.Add(duration_cast(currentTime - start));
			last = true;
		}
		lastTime = currentTime;
	}

	void CallbackGotString(const std::string& msg, const Time& currentTime = Now()) override {
		//error
	}

	void CallbackMsgQueueEmpty(const Time& currentTime = Now()) override {
		//printf(" QueueEmpty %lld\n", duration_cast(currentTime.time_since_epoch()).count());
		if (!last)
			wrongqueueended++;
		if (last) {
			afterfifthmsg.Add(duration_cast(currentTime - lastTime));
			last = false;
		}
		lastTime = currentTime;
	}


	void printStatistics() const {
		std::cout << "between 5 :\n max: " << between5msgs.GetMax().count() << " us\n";
		std::cout << " average: " << between5msgs.GetAverage().count() << " us\n";
		std::cout << "after 5. msg :\n max: " << afterfifthmsg.GetMax().count() << " us\n";
		std::cout << " average: " << afterfifthmsg.GetAverage().count() << " us\n";
		std::cout << "sampling time 5. msg :\n max: " << samplingTime.GetMax().count() << " us\n";
		std::cout << " maxK: " << samplingTime.GetMaxK().count() << " us\n";
		std::cout << " average: " << samplingTime.GetAverage().count() << " us\n";
		std::cout << "Missing closing msg: " << errors << "\nUnnecessary closing msgs: " << wrongqueueended << std::endl;
	}

	void Evaluate() const {
		TEST_ASSERT(between5msgs.GetMax() < 5 * 2 * std::chrono::microseconds(100));
		TEST_ASSERT(between5msgs.GetAverage() < 5 * std::chrono::microseconds(100));

		TEST_ASSERT(afterfifthmsg.GetMax() < Reciever::tWaitNextMsg * 1.3);
		TEST_ASSERT(afterfifthmsg.GetMax() > Reciever::tWaitNextMsg * 0.7);
		TEST_ASSERT(afterfifthmsg.GetAverage() < Reciever::tWaitNextMsg * 1.15);
		TEST_ASSERT(afterfifthmsg.GetAverage() > Reciever::tWaitNextMsg * 0.85);
	}

	void Evaluate(DTime Ts) const {
		Evaluate();
		TEST_ASSERT(samplingTime.GetMax() < Ts * 2);
		TEST_ASSERT(samplingTime.GetMax() > Ts * 0.7);
		TEST_ASSERT(samplingTime.GetMaxK() < Ts * 1.2);
		TEST_ASSERT(samplingTime.GetMaxK() > Ts * 0.8);
		TEST_ASSERT(samplingTime.GetAverage() < Ts * 1.1);
		TEST_ASSERT(samplingTime.GetAverage() > Ts * 0.9);
	}
};

class AppLayerM : public AppLayer {
public:
	void CallbackSamplingTimeOver(const Time& currentTime = Now()) override {
		CallbackGotString("SamplingTimeOver", currentTime);
	}

	void CallbackMsgQueueEmpty(const Time& currentTime = Now()) override {
		CallbackGotString("MsgQueueEmpty", currentTime);
	}
};

void test_periphery_logging_via_CU() {
	std::string filename("test_periphery_" + std::to_string(duration_cast(Now().time_since_epoch()).count()) + ".txt");
	DTime Ts = DTime(1000 * 25);
	{
		NetworkConfig n;
		for (int i = 0; i < 5; i++)
			n.Add(std::to_string(i), NetworkConfig::ConnectionData("tcp", "localhost", "234" + std::to_string(i)));

		CentralUnit l(filename, std::make_shared<AppLayerM>(), n);
		
		l.Start(Ts);

		std::this_thread::sleep_for(std::chrono::seconds(1));

		std::shared_ptr<Periphery> p[5];
		for (int i = 0; i < 5; i++)
			p[i] = std::make_shared<Periphery>(n.GetPeripheryData(std::to_string(i)));

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		DTime Twait(10000);
		Time tNext = Now() + Twait;
		for (int i = 0; i < 15; i++) {
			for (int j = 0; j < 5; j++)
				p[j]->SendValue(i * 10 + j, Eigen::VectorXd::Ones(j * 10)*i*0.99999, OUTPUT);
			while (Now() < tNext)
				;
			tNext += Twait;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}
	Tester t;
	{
		struct Row {
			enum Type { DATAMSG, QUEUEENDED, SAMPLINGOVER} type;
			Time time;
		};
		SPDLogReader reader(filename);
		std::vector<Row> rows = std::vector<Row>();
		while (reader.readNextRow() != Reciever::NOTHING) {
			switch (reader.getLatestRowType()) {
			case Reciever::DATAMSG:
				t.CallbackGotDataMsg(DataMsg(), reader.getLatestTimeStamp());
				break;
			case Reciever::TEXT:
				if (reader.getLatestRowIf().compare("MsgQueueEmpty") == 0)
					t.CallbackMsgQueueEmpty(reader.getLatestTimeStamp());
				if (reader.getLatestRowIf().compare("SamplingTimeOver") == 0)
					t.CallbackSamplingTimeOver(reader.getLatestTimeStamp());
				break;
			}
		}
		t.printStatistics();
	}
	// delete created log file
	if (remove(filename.c_str()) != 0)
		perror("Error deleting file");
	t.Evaluate(Ts);
}



void test_peripheries_logger_centralunitemulator() {
	std::string filename("test_periphery2_" + std::to_string(duration_cast(Now().time_since_epoch()).count()) + ".txt");
	// simulate 5 sensors sending msgs to a logger
	{
		NetworkConfig n;
		for (int i = 0; i < 5; i++)
			n.Add(std::to_string(i), NetworkConfig::ConnectionData("tcp", "localhost", "234" + std::to_string(i)));
		Logger l(filename);
		l.AddPeripheries(n);
		l.Start(DTime(1000 * 25));

		std::this_thread::sleep_for(std::chrono::seconds(1));

		std::shared_ptr<Periphery> p[5];
		for (int i = 0; i < 5; i++)
			p[i] = std::make_shared<Periphery>(n.GetPeripheryData(std::to_string(i)));

		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		DTime Ts(10000);
		Time tNext = Now() + Ts;
		for (int i = 0; i < 15; i++) {
			for (int j = 0; j < 5; j++)
				p[j]->SendValue(i * 10 + j, Eigen::VectorXd::Ones(j * 10)*i*0.99999, OUTPUT);
			while (Now() < tNext)
				;
			tNext += Ts;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}
	std::shared_ptr<Tester> tester1 = std::make_shared<Tester>();
	DTime Ts1(1000 * 15);
	{
		CentralUnitEmulator unit(false, tester1, filename);
		unit.Start(Ts1);
		while (unit.isRunning())
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	std::shared_ptr<Tester> tester2 = std::make_shared<Tester>();
	DTime Ts2(1000 * 15);
	{
		CentralUnitEmulator unit(true, tester2, filename);
		unit.Start(Ts2);
		while (unit.isRunning())
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	// delete created log file
	if (remove(filename.c_str()) != 0)
		perror("Error deleting file");

	tester1->printStatistics();
	tester1->Evaluate(Ts1);

	tester2->printStatistics();
	//tester2->Evaluate(Ts2);
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() { test_periphery_logging_via_CU(); });
	RUN_TEST([]() { test_peripheries_logger_centralunitemulator(); });
	/*
	RUN_TEST([]() { orderDataMsg("tcp://*:1234", "tcp://localhost:1234", 100); });
	RUN_TEST([]() { orderStrings("tcp://*:1234", "tcp://localhost:1234", 100); });
	RUN_TEST([]() {
		printf("TCP: 1000x5 datamsg\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 1000, 5);
		printf("TCP: 1000x5 string\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 1000, 5, true);
		printf("TCP: 10000x5 datamsg\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 10000, 5);
		printf("TCP: 10000x5 string\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 10000, 5, true);
	});
	RUN_TEST([]() {	hwmtest("tcp://*:1234", "tcp://localhost:1234", 10, 10); });
	RUN_TEST([]() {	hwmtest("tcp://*:1234", "tcp://localhost:1234", 10, 100); });
	RUN_TEST([]() {	DataMsgContentSerialization(100000); });
	
#ifdef UNIX
	//ipc, inproc...
	RUN_TEST([]() {
		printf("1000,5, datamsg\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 1000, 5);
		printf("1000,5, string\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 1000, 5, true);
		printf("10000,5\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 10000, 5);
		printf("10000,5, string\n");
		SendAndRecieveDataMsgs("tcp://*:1234", "tcp://localhost:1234", 10000, 5, true);
	});
#endif
*/
	return UNITY_END();
}
