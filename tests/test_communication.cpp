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

void test_periphery_logger() {
	std::string filename("test_periphery.txt");
	{
		
		Logger l(filename);
		for (int i = 0; i < 5; i++)
			l.AddPeriphery(Reciever::PeripheryProperties("tcp://localhost:234" + std::to_string(i), true));
		l.Start(DTime(1000 * 10));

		std::this_thread::sleep_for(std::chrono::seconds(1));

		std::shared_ptr<Periphery> p[5];
		for (int i = 0; i < 5; i++) {
			p[i] = std::make_shared<Periphery>("tcp://*:234" + std::to_string(i), 10);
		}

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
	{
		struct Row {
			enum Type { DATAMSG, QUEUEENDED, SAMPLINGOVER} type;
			Time time;
		};
		SPDLogReader reader(filename);
		std::vector<Row> rows = std::vector<Row>();
		while (reader.readNextRow() != MsgType::NOTHING) {
			Row row;
			row.time = reader.getLatestTimeStamp();
			if (reader.getLatestRowType() == MsgType::DATAMSG) {
				row.type = Row::DATAMSG;
			}
			if (reader.getLatestRowType() == MsgType::TEXT) {
				if (reader.getLatestRowIf().compare("MsgQueueEmpty") == 0)
					row.type = Row::QUEUEENDED;
				if (reader.getLatestRowIf().compare("SamplingTimeOver") == 0)
					row.type = Row::SAMPLINGOVER;
			}
			rows.push_back(row);
		}
		// Check max elapsed time between the 5 msgs
		DTime maxbetween5DataMsgs(0);
		int j = 0;
		Time start;
		for (int i=0;i<rows.size();i++) {
			if (rows[i].type == Row::DATAMSG) {
				if (j % 5 == 0)
					start = rows[i].time;
				j++;
				if (j % 5 == 0) {
					DTime temp = duration_cast(rows[i].time - start);
					maxbetween5DataMsgs = maxbetween5DataMsgs > temp ? maxbetween5DataMsgs : temp;
				}
			}
		}
		std::cout << "max: " << maxbetween5DataMsgs.count() << " us\n";
		// Check max time between two "SamplingTimeOver"


	}
	// delete created log file
	if (remove(filename.c_str()) != 0)
		perror("Error deleting file");
}

/*
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

void SendAndRecieveDataMsgs(std::string senderaddress, std::string recvaddress, int N, int K, bool sendstring = false) {
	DataMsg d(1, STATE, SENSOR, Now());
	d.SetValueVector(Eigen::VectorXd::Ones(10));
	d.SetVarianceMatrix(Eigen::MatrixXd::Identity(10, 10));
	ZMQSender a(senderaddress, N);
	ZMQReciever r;
	r.AddPeriphery(ZMQReciever::PeripheryProperties(OperationType::SENSOR, 1, DataType::STATE,
		recvaddress, true));
	r.Start(DTime(1000));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	for (int k = 0; k < K; k++) {
		Time start = Now();
		for (long long n = 0; n < N; n++)
			if (sendstring)
				a.SendString("abasdasdfasf");
			else
				a.SendDataMsg(d);
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
	ZMQSender a(senderaddress, hwm);
	ZMQReciever r;
	r.AddPeriphery(ZMQReciever::PeripheryProperties(OperationType::SENSOR, 1, DataType::STATE,
		recvaddress, true));
	r.Start(DTime(1000));
	long long recieved = 0;
	for (int j = 0; j < N; j++) {
		r.Pause(true);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		for (int i = 0; i < hwm*100; i++)
			a.SendDataMsg(d);
		
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
class Checker : public Processor {
	int n = 0;
public:
	void CallbackGotDataMsg(const DataMsg& msg, const Time& currentTime = Now()) override {
		if (msg != msgs[n]) {
			printf("Error:\n");
			msg.print();
			msgs[n].print();
		}
		error |= (msg != msgs[n]);
		n++;
	}

	void CallbackGotString(const std::string& msg, const Time& currentTime = Now()) override {
		error |= (msg.compare(strings[n]) != 0);
		n++;
	}
};

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
	ZMQSender a(senderaddress, N);
	auto checker = std::make_shared<Checker>();
	ZMQReciever r;
	r.SetProcessor(checker);
	r.AddPeriphery(ZMQReciever::PeripheryProperties(recvaddress, true));
	r.Start(DTime(1000));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	for (int i = 0; i < N; i++)
		a.SendDataMsg(msgs[i]);

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
	ZMQSender a(senderaddress, N);
	auto checker = std::make_shared<Checker>();
	ZMQReciever r;
	r.SetProcessor(checker);
	r.AddPeriphery(ZMQReciever::PeripheryProperties(recvaddress, true));
	r.Start(DTime(1000));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	for (int i = 0; i < N; i++)
		a.SendString(strings[i]);

	std::this_thread::sleep_for(std::chrono::milliseconds(1));

	while (r.GetNumOfRecievedMsgs(0) != N)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	r.Stop();
	if (error)
		TEST_ASSERT(0);
}*/

int main (void) {

	test_periphery_logger();



	UNITY_BEGIN();
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
