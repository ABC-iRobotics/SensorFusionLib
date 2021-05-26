#include "common/unity.h"
void setUp() {}
void tearDown() {}

#include <thread>
#include <iostream>
#include"SystemManager.h"

using namespace SF;


class SystemManagerTester : public SystemManager {
public:
	SystemManagerTester(const BaseSystemData& data, const StatisticValue& state_) : SystemManager(data,state_) {}

	void SamplingTimeOver(const Time& currentTime) override {}; /*!< Is called in each sampling time - input: time */

	void MsgQueueEmpty(const Time& currentTime) override {}; /*!< Is called if the DataMsgs in the queue were read */

};

class TestBaseSystem : public BaseSystem {
public:
	TestBaseSystem() : BaseSystem(0) {}

	Eigen::MatrixXd getA(double Ts) const {
		Eigen::MatrixXd out = Eigen::MatrixXd::Identity(6,6);
		out(2, 2) = 0;
		out(3, 3) = 0;
		out(3, 2) = 1;
		out(5, 5) = 0;
		return out;
	}

	Eigen::MatrixXd getB(double Ts) const {
		Eigen::MatrixXd out = Eigen::MatrixXd::Zero(6, 3);
		out(0, 0) = 1;
		out(1, 1) = 1;
		return out;
	}

	Eigen::MatrixXd getC(double Ts) const {
		return Eigen::MatrixXd(0, 6);
	}

	Eigen::MatrixXd getD(double Ts) const {
		return Eigen::MatrixXd(0, 3);
	}

	// The nonlinear parts and the dependencies are by default zeros
	Eigen::VectorXi getStateUpdateNonlinXDep() const {
		Eigen::VectorXi out = Eigen::VectorXi::Zero(6);
		out[2] = 1;
		out[3] = 1;
		out[5] = 1;
		return out;
	}

	Eigen::VectorXi getStateUpdateNonlinWDep() const {
		Eigen::VectorXi out(3);
		out[0] = 1;
		out[1] = 1;
		out[2] = 0;
		return out;
	}

	Eigen::VectorXd EvalStateUpdateNonlinearPart(double Ts,
		const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const {
		Eigen::VectorXd out(6);
		out[0] = -disturbance[0];
		out[1] = -disturbance[1];
		out[2] = state[2];
		out[3] = -state[2] + state[3];
		out[4] = 0;
		out[5] = state[5];
		return out;
	}

	unsigned int getNumOfStates() const { return 6; }

	unsigned int getNumOfDisturbances() const { return 3; }

	unsigned int getNumOfOutputs() const { return 0;  }

	unsigned int getNumOfNoises() const { return 0; }
};

void linearTest() {
	Eigen::MatrixXd Sw = Eigen::MatrixXd::Zero(3, 3);
	Sw(0, 0) = 5;
	Sw(1, 1) = 10;
	Sw(1, 1) = 7;

	StatisticValue in(Eigen::VectorXd::Zero(3), Sw);

	SystemManagerTester::BaseSystemData bsData(std::make_shared<TestBaseSystem>(), StatisticValue(0),
		in);

	Eigen::MatrixXd Sx = Eigen::MatrixXd::Identity(6, 6);

	StatisticValue state(Eigen::VectorXd::Zero(6), Sx);

	SystemManagerTester tester(bsData, state);

	StatisticValue y;
	Eigen::MatrixXd Syin, Syx;

	y = tester.Eval(TimeUpdateType::STATE_UPDATE, 0.001, state, in, Syx, Syin);

	std::cout << y << std::endl << std::endl;
	std::cout << Syx << std::endl << std::endl;
	std::cout << Syin << std::endl << std::endl;

	if (y.vector.cwiseAbs().sum() > 1e-7)
		TEST_ASSERT(0);
	if ((y.variance-Eigen::MatrixXd::Identity(6,6)).cwiseAbs().sum() > 1e-7)
		TEST_ASSERT(0);
	if ((Syx - Eigen::MatrixXd::Identity(6, 6)).cwiseAbs().sum() > 1e-7)
		TEST_ASSERT(0);
	if (Syin.cwiseAbs().sum() > 1e-7)
		TEST_ASSERT(0);
}

int main (void) {
	UNITY_BEGIN();
	RUN_TEST([]() { linearTest(); });
	return UNITY_END();
}
