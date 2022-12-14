#include "common/unity.h"
void setUp() {}
void tearDown() {}

#include "Forwarder.h"
#include "SPDLogReader.h"
#include <iostream>

using namespace SF;

void test_speed(int Ndata, int Ncases, int TsUSassert, int TsUSwarning) {

	DataMsg msg = DataMsg(5, OUTPUT, SENSOR);
	Eigen::VectorXd v(3); v(0) = 2; v(1) = 1; v(2) = 0;
	msg.SetValueVector(v);
	Eigen::MatrixXd m = Eigen::MatrixXd::Identity(3, 3) / 1000.;
	msg.SetVarianceMatrix(m);
	double a = 0.001;
	std::string filename = "test_log_" + std::to_string(Now().time_since_epoch().count()) + ".txt";
	{
		std::vector<double> results = std::vector<double>();
		Forwarder logger;
		logger.SetLogger(filename.c_str());
		for (int n = 0; n < Ncases; n++) {
			auto start = Now();
			for (long int i = 0; i < Ndata; i++)
				logger.ForwardDataMsg(msg,Now());
			auto elapsed = Now() - start;

			double delapsed = (double)duration_cast(elapsed).count() / (double)Ndata;
			TEST_ASSERT_LESS_THAN(TsUSassert, delapsed);
			if (delapsed > TsUSwarning)
				std::cout << "Warning writing one DataMsg took " << delapsed << " us in average!\n";
			results.push_back(delapsed);
		}
		std::cout << "Average writing times: ";
		for (int i = 0; i < results.size(); i++)
			std::cout << results[i] << " us   ";
		std::cout << std::endl;
	}
	{
		std::vector<double> results = std::vector<double>();
		DataMsg msg;
		auto reader = SPDLogReader(filename.c_str());
		for (int n = 0; n < Ncases; n++) {
			auto start = Now();
			for (long int i = 0; i < Ndata; i++) {
				if (reader.getLatestRowType() != DATAMSG)
					TEST_ASSERT(false);
				msg = reader.getLatestDataMsgIf();
				reader.readNextRow();
			}
			auto elapsed = Now() - start;

			double delapsed = (double)duration_cast(elapsed).count() / (double)Ndata;
			TEST_ASSERT_LESS_THAN(TsUSassert, delapsed);
			if (delapsed > TsUSwarning)
				std::cout << "Warning reading one DataMsg took " << delapsed << " us in average!\n";
			results.push_back(delapsed);
		}
		std::cout << "Average reading times: ";
		for (int i = 0; i < results.size(); i++)
			std::cout << results[i] << " us   ";
		std::cout << std::endl;
	}
	// delete created log file
	if (remove(filename.c_str()) != 0)
		perror("Error deleting file");
}

Time RandTime() {
	long long temp = duration_cast(Now().time_since_epoch()).count() % 1000000;
	return Time(DTime(temp*temp));
}

void GenerateMessages(int Ndata, DataMsg::DataMsgPtrList& out) {
	// Generate datamsgs
	Eigen::VectorXd t = Eigen::VectorXd::Zero(4);
	t(2) = 2;
	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(4, 4);
	T(1, 2) = 3;
	T(2, 1) = 3;
	for (int n = 0; n < Ndata; n++) {
		auto value1 = Eigen::VectorXd::Ones(4) / double(n + 1);
		auto value2 = t * (n - 1);
		auto variance1 = Eigen::MatrixXd::Ones(2, 2) / double(n + 1);
		auto variance2 = T * (n - 1);

		auto a = DataMsg::CreateSharedPtr(n, DataType::OUTPUT, OperationType::FILTER_MEAS_UPDATE, RandTime());
		a->SetValueVector(value1);
		out.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::STATE, OperationType::FILTER_PARAM_ESTIMATION, RandTime());
		a->SetValueVector(value2);
		a->SetVarianceMatrix(variance2);
		out.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::DISTURBANCE, OperationType::FILTER_TIME_UPDATE, RandTime());
		a->SetVarianceMatrix(variance2);
		out.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::INVALID_DATATYPE, OperationType::GROUND_TRUTH, RandTime());
		a->SetVarianceMatrix(variance1);
		out.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::NOISE, OperationType::INVALID_OPERATIONTYPE, RandTime());
		out.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::DISTURBANCE, OperationType::SENSOR, RandTime());
		a->SetValueVector(value1);
		out.push_back(a);
	}
}

void test_read_write(int Ndata) {
	// Wtite 1000 different messages N times into a file, read them, measuring the elapsed time
	auto msgs = DataMsg::DataMsgPtrList();
	GenerateMessages(Ndata, msgs);
	std::string filename = "read_write_test_" + std::to_string(Now().time_since_epoch().count()) + ".txt";
	// Send them into a new logger
	{
		Forwarder w;
		w.SetLogger(filename.c_str());
		for (int i = 0; i < msgs.size(); i++)
			w.ForwardDataMsg(*msgs[i],Now());
	}
	// Read the log - checking the results...
	bool ok = true;
	{
		SPDLogReader r(filename);
		int i = 0;
		while (r.getLatestRowType() == DATAMSG) {
			auto msg = r.getLatestDataMsgIf();
			if (msg != *msgs[i])
				ok = false;
			i++;
			r.readNextRow();
		}
	}
	// delete created log file
	if (remove(filename.c_str()) != 0)
		perror("Error deleting file");
	TEST_ASSERT(ok);
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() {test_speed(1000, 5, 1000, 10); });
	RUN_TEST([]() {test_read_write(1000); });
	return UNITY_END();
}
