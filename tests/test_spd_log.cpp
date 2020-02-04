#include "Common/unity.h"
void setUp() {}
void tearDown() {}

#include "spdLogWrite.h"
#include "spdLogRead.h"
#include <iostream>

using namespace SF;

void test_speed(int Ndata, int Ncases, int TsUSassert, int TsUSwarning) {

	DataMsg msg = DataMsg(5, OUTPUT, SENSOR);
	Eigen::VectorXd v(3); v(0) = 2; v(1) = 1; v(2) = 0;
	msg.SetValueVector(v);
	Eigen::MatrixXd m = Eigen::MatrixXd::Identity(3, 3) / 1000.;
	msg.SetVarianceMatrix(m);
	double a = 0.001;
	std::string filename = "test_log.txt";
	{
		std::vector<double> results = std::vector<double>();
		spdLogWrite logger(filename.c_str(), "log_tester");
		for (int n = 0; n < Ncases; n++) {
			auto start = SF::Now();
			for (long int i = 0; i < Ndata; i++)
				logger.WriteDataMsg(msg);
			auto elapsed = SF::Now() - start;

			double delapsed = (double)SF::duration_cast(elapsed).count() / (double)Ndata;
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
		auto reader = spdLogRead(filename.c_str());
		for (int n = 0; n < Ncases; n++) {
			auto start = SF::Now();
			for (long int i = 0; i < Ndata; i++) {
				reader.readNextRow();
				if (reader.getLatestRowType() != reader.DATAMSG)
					TEST_ASSERT("Read error!");
				msg = reader.getLatestDataMsgIf();
			}
			auto elapsed = SF::Now() - start;

			double delapsed = (double)SF::duration_cast(elapsed).count() / (double)Ndata;
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

bool IsEqual(const DataMsg& d1, const DataMsg& d2) {
	if (d1.IsEmpty() != d2.IsEmpty())
		return false;
	if (d1.HasValue() != d2.HasValue())
		return false;
	if (d1.HasVariance() != d2.HasVariance())
		return false;
	if (d1.GetSourceID() != d2.GetSourceID())
		return false;
	if (d1.GetDataSourceType() != d2.GetDataSourceType())
		return false;
	if (d1.GetDataType() != d2.GetDataType())
		return false;
	if (d1.GetTime() != d2.GetTime())
		return false;
	if (d1.HasValue())
		if (d1.GetValue() != d2.GetValue())
			return false;
	if (d1.HasVariance())
		if (d1.GetVariance() != d2.GetVariance())
			return false;
	return true;
}

void test_read_write(int Ndata) {
	// Wtite 1000 different messages N times into a file, read them, measuring the elapsed time
	// Generate datamsgs
	Eigen::VectorXd t = Eigen::VectorXd::Zero(4);
	t(2) = 2;
	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(4, 4);
	T(1, 2) = 3;
	auto msgs = DataMsg::DataMsgPtrList();
	std::string filename = "read_write_test.txt";
	for (int n = 0; n < Ndata; n++) {
		auto value1 = Eigen::VectorXd::Ones(4) / double(n + 1);
		auto value2 = t*(n - 1);
		auto variance1 = Eigen::MatrixXd::Ones(2, 2) / double(n + 1);
		auto variance2 = T*(n - 1);
		auto a = DataMsg::CreateSharedPtr(n, DataType::OUTPUT, OperationType::FILTER_MEAS_UPDATE, Time());
		a->SetValueVector(value1);
		msgs.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::STATE, OperationType::FILTER_PARAM_ESTIMATION, Time());
		a->SetValueVector(value2);
		a->SetVarianceMatrix(variance2);
		msgs.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::DISTURBANCE, OperationType::FILTER_TIME_UPDATE, Time());
		a->SetVarianceMatrix(variance2);
		msgs.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::INVALID_DATATYPE, OperationType::GROUND_TRUTH, Time());
		a->SetVarianceMatrix(variance1);
		msgs.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::NOISE, OperationType::INVALID_OPERATIONTYPE, Time());
		msgs.push_back(a);
		a = DataMsg::CreateSharedPtr(n, DataType::DISTURBANCE, OperationType::SENSOR, Time());
		a->SetValueVector(value1);
		msgs.push_back(a);
	}
	// Send them into a new logger
	{
		spdLogWrite w(filename, "read_write_test");
		for (int i = 0; i < msgs.size(); i++)
			w.WriteDataMsg(*msgs[i]);
	}
	// Read the log - checking the results...
	{
		spdLogRead r(filename);
		int i = 0;
		while (r.getLatestRowType() == spdLogRead::DATAMSG) {
			auto msg = r.getLatestDataMsgIf();
			if (!IsEqual(msg, *msgs[i]))
				TEST_ASSERT("Written and read msgs are different!");
			i++;
		}
	}
	// delete created log file
	if (remove(filename.c_str()) != 0)
		perror("Error deleting file");
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() {test_speed(1000, 5, 1000, 10); });
	RUN_TEST([]() {test_read_write(1000); });
	return UNITY_END();
}
