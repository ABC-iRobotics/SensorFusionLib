#include "common/unity.h"
void setUp() {}
void tearDown() {}

#include <thread>
#include <iostream>
#include"msgcontent2buf.h"
#include "FilterCore.h"

using namespace SF;

void DataMsgContentSerialization(long N) {
	for (long long i = 0; i < N; i++) {
		unsigned char* buf;
		int length;
		OperationType source(SENSOR);
		DataType type(OUTPUT);
		unsigned char ID = 5;
		Time t(Now());
		DataMsg d(ID, type, source, t);
		d.SetValueVector(Eigen::VectorXd::Ones(10));
		d.SetVarianceMatrix(Eigen::MatrixXd::Identity(10, 10));
		SerializeDataMsg(d, buf, length);
		if (VerifyDataMsgContent(buf, length)) {
			TEST_ASSERT(InitDataMsg(buf, source, ID, type) == d);
		}
		else
			printf("Error...");
		delete buf;
	}
}
#include "Forwarder.h"
#include "ZMQReciever.h"

class ZMQRecievingTest : public ZMQReciever {
	void SamplingTimeOver(const Time& currentTime) override {

	}

	void SaveDataMsg(const DataMsg& msg, const Time& currentTime) override {

	}

	void MsgQueueEmpty(const Time& currentTime) override {}

	void SaveString(const std::string& msg, const Time& currentTime) override {}

};


void SendAndRecieveDataMsgs(std::string senderaddress, std::string recvaddress, int N, int K, bool sendstring = false) {
	DataMsg d(1, STATE, SENSOR, Now());
	d.SetValueVector(Eigen::VectorXd::Ones(10));
	d.SetVarianceMatrix(Eigen::MatrixXd::Identity(10, 10));
	Forwarder a;
	a.SetZMQOutput(senderaddress.c_str(), N);
	ZMQRecievingTest r;
	r.AddPeriphery(ZMQRecievingTest::PeripheryProperties(OperationType::SENSOR, 1, DataType::STATE,
		recvaddress, true));
	r.Start(DTime(1000));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	for (int k = 0; k < K; k++) {
		Time start = Now();
		for (long long n = 0; n < N; n++)
			if (sendstring)
				a.ForwardString("abasdasdfasf", Now());
			else
				a.ForwardDataMsg(d, Now());
		while (r.GetNumOfRecievedMsgs(0) != N * (k + 1))
			std::this_thread::sleep_for(std::chrono::microseconds(1));
		DTime dt = duration_cast(Now() - start);
		printf("MSGs got: Nx%lli us (=%lli us)\n", dt.count() / N, dt.count());
	}
}

void hwmtest(std::string senderaddress, std::string recvaddress, int N, int hwm) {
	DataMsg d(1, STATE, SENSOR, Now());
	d.SetValueVector(Eigen::VectorXd::Ones(10));
	d.SetVarianceMatrix(Eigen::MatrixXd::Identity(10, 10));
	Forwarder a;
	a.SetZMQOutput(senderaddress.c_str(), N);
	ZMQRecievingTest r;
	r.AddPeriphery(ZMQRecievingTest::PeripheryProperties(OperationType::SENSOR, 1, DataType::STATE,
		recvaddress, true));
	r.Start(DTime(1000));
	long long recieved = 0;
	for (int j = 0; j < N; j++) {
		r.Pause(true);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		for (int i = 0; i < hwm*100; i++)
			a.ForwardDataMsg(d,Now());
		
		long long recieved0 = r.GetNumOfRecievedMsgs(0);
		r.Pause(false);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		while (r.GetNumOfRecievedMsgs(0) != recieved) {
			recieved = r.GetNumOfRecievedMsgs(0);
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		printf("  n: %lli, %f\n", recieved - recieved0, (float)(recieved-recieved0)/float(hwm));
	}
	printf("DONE \n");
}

bool error;
static std::vector<DataMsg> msgs;
static std::vector<std::string> strings;
class Checker : public FilterCore {
	int n = 0;
public:
	size_t nSensors() const {
		return 0;
	}

	bool SaveDataMsg(const DataMsg& msg, const Time& currentTime = Now()) override {
		if (msg != msgs[n]) {
			printf("Error:\n");
			msg.print();
			msgs[n].print();
		}
		error |= (msg != msgs[n]);
		n++;
		return true;
	}

	void SamplingTimeOver(const Time& currentTime) {}

	void MsgQueueEmpty(const Time& currentTime) {}

	DataMsg GetDataByID(int systemID, DataType dataType, OperationType opType) override {
		return DataMsg();
	}

	DataMsg GetDataByIndex(int systemIndex, DataType dataType, OperationType opType) override {
		return DataMsg();
	}
};

#include "Filter.h"

void orderDataMsg(std::string senderaddress, std::string recvaddress, int N) {
	error = false;
	msgs = std::vector<DataMsg>();
	for (int i = 0; i < N; i++) {
		DataMsg d(5, DataType(i % 4), OperationType(i % 3), Now());
		if (i%3==0)
			d.SetValueVector(Eigen::VectorXd::Ones(4)*i);
		if (i % 2 == 0)
			d.SetVarianceMatrix(Eigen::MatrixXd::Ones(i%6,i%6)*i);
		msgs.push_back(d);
	}
	Forwarder a;
	a.SetZMQOutput(senderaddress.c_str(), N);
	auto checker = std::make_shared<Checker>();
	Filter r(checker);
	r.AddPeriphery(Filter::PeripheryProperties(recvaddress, true));
	r.Start(DTime(1000));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	for (int i = 0; i < N; i++)
		a.ForwardDataMsg(msgs[i],Now());

	std::this_thread::sleep_for(std::chrono::milliseconds(1));

	while (r.GetNumOfRecievedMsgs(0) != N)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	r.Stop();
	if (error)
		TEST_ASSERT(0);
}

void orderStrings(std::string senderaddress, std::string recvaddress, int N) {
	error = false;
	strings = std::vector<std::string>();
	for (int i = 0; i < N; i++)
		strings.push_back(std::string("asdassv0") + std::to_string(i) + "_" + std::to_string(duration_cast(Now().time_since_epoch()).count()));
	Forwarder a;
	a.SetZMQOutput(senderaddress.c_str(), N);
	auto checker = std::make_shared<Checker>();
	Filter r(checker);
	r.AddPeriphery(Filter::PeripheryProperties(recvaddress, true));
	r.Start(DTime(1000));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	for (int i = 0; i < N; i++)
		a.ForwardString(strings[i],Now());

	std::this_thread::sleep_for(std::chrono::milliseconds(1));

	while (r.GetNumOfRecievedMsgs(0) != N)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	r.Stop();
	if (error)
		TEST_ASSERT(0);
}

int main (void) {
	UNITY_BEGIN();
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
	return UNITY_END();
}
