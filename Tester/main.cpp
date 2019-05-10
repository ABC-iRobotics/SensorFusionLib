// SensorFusion.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <thread>
#include "FilterPlot.h"
#include "KalmanFilter.h"
#include "youBotSystem.h"
#include "IMUSensor.h"

/*
void FilterCallback(FilterCallData data) {
	if (data.callType == FilterCallData::FILTERING && data.type == STATE) {
		std::cout << data.value.vector.transpose() << std::endl;
	}
}
*/

SystemManager::BaseSystemData InitYouBotSystem2(BaseSystem::BaseSystemPtr ptr, StatisticValue& initState) {
	// Init state (vx,vy,om,x,y,phi,null)
	initState = StatisticValue(Eigen::VectorXd::Zero(7), Eigen::MatrixXd::Identity(7, 7)*1.1);
	// Init noise ()
	StatisticValue initNoise(0);
	// Init disturbance (FL,FR,BL,BR) : must be updated later
	StatisticValue initDist(Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Identity(4, 4)*0.01);
	// Measurement: always zero
	Eigen::VectorXd constantMeasurement = Eigen::VectorXd::Zero(1);
	return SystemManager::BaseSystemData(ptr, initNoise, initDist, constantMeasurement, SystemManager::CONSTANT);
}

SystemManager::SensorData InitIMUSensor2(Sensor::SensorPtr ptr, StatisticValue& initState) {
	// Init state (vx_old,vy_old,sx,sy,som)
	initState = StatisticValue(Eigen::VectorXd::Zero(2), Eigen::MatrixXd::Identity(2, 2)*2.);
	initState.Add(StatisticValue(Eigen::VectorXd::Ones(3), Eigen::MatrixXd::Identity(3, 3)*0.1));
	// Init noise (bx,by,bom)
	StatisticValue initNoise(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)*1.1);
	// Init disturbance () 
	StatisticValue initDist(0);
	return SystemManager::SensorData(ptr, initNoise, initDist);
}

int main() {

	BaseSystem::BaseSystemPtr youBot = std::make_shared<youBotSystem>(0.002, 0.4, 0.25, 0.05);
	StatisticValue initState;
	SystemManager::BaseSystemData data2 = InitYouBotSystem2(youBot, initState);
	KalmanFilter filter(data2, initState);

	Sensor::SensorPtr imu = std::make_shared<IMUSensor>(youBot);
	SystemManager::SensorData data3 = InitIMUSensor2(imu, initState);
	filter.AddSensor(data3, initState);

	//filter.AddCallback(FilterCallback, 0);
	FilterPlot plotter(filter, imu, OUTPUT);
	FilterPlot plotter2(filter, imu, STATE);
	FilterPlot plotter3(filter, youBot, STATE);
	
	Eigen::VectorXd a = Eigen::VectorXd(3);
	a(0) = 0.1; a(1) = 0.3; a(2) = 0.4;
	for (; true;) {
		imu->MeasurementDone(a);
		filter.Step(0.001);
		std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100.));
		FilterPlot::UpdateWindows();
	}

	return 0;


	/*
	std::cout << man.EvalNonLinPart(0.001, System::TIMEUPDATE, man(STATE).vector, man(DISTURBANCE).vector) << std::endl << std::endl;

	std::cout << man.EvalNonLinPart(0.001, System::MEASUREMENTUPDATE, man(STATE).vector, man(NOISE).vector) << std::endl << std::endl;

	return 0;
	*/

	//imu5->MeasurementDone(a);
	/*
	std::cout << man.num(DISTURBANCE) << std::endl;
	std::cout << man.num(STATE) << std::endl;
	std::cout << man.num(NOISE) << std::endl;
	std::cout << man.num(OUTPUT) << std::endl;




	SystemValueType type = OUTPUT;
	std::cout << man(type).vector << std::endl << std::endl;
	std::vector<Eigen::VectorXd> temp = man.partitionate(type, man(type).vector);
	for (int i = 0; i < temp.size(); i++)
		std::cout << temp[i] << std::endl << std::endl;

	//return 0;
	std::cout << man(STATE) << std::endl << std::endl;

	std::cout << A << std::endl << std::endl;

	std::cout << B << std::endl << std::endl;

	return 0;


	FilterSystem::SystemData data = InitYouBotSystem(youBot, initState);
	FilterSystem sys(data, initState);

	data = InitIMUSensor(imu, initState);
	sys.AddSensor(data, initState);

	

	// Usage
	std::cout << sys;

	sys.Step(0.001);

	std::cout << sys;

	std::cout << sys;

	sys.Step(0.001);

	std::cout << sys;

	return 0;

	//std::cout << "New state:\n" << sys.ComputeOutput(0.001, state,) << std::endl;


	imu->MeasurementDone(Eigen::VectorXd(3));


	std::cout << "2\n";
	imu->MeasurementDone(Eigen::VectorXd(3));
	std::cout << "3\n";

	FilterPlotter plotter(sys, FilterPlotter::STATE);
	sys.Step(0.01);


	/*

	imu->MeasurementDone(Eigen::VectorXd(3));
	std::cout << "2\n";
	imu->MeasurementDone(Eigen::VectorXd(3));
	std::cout << "3\n";



	/*
	bool exit = false;


	auto start = std::chrono::steady_clock::now();
	/*
	while (exit == false)
	{
		auto actual = std::chrono::steady_clock::now();


		if (GetAsyncKeyState(VK_ESCAPE))
		{
			exit = true;
		}
		cvplot::figure("myplot").series("myline")
			.addValue((float)std::chrono::duration_cast<std::chrono::milliseconds>(actual-start).count());
		cvplot::figure("myplot").show();
		start = actual;
	}
	*/
	
/*
	unsigned int i;
	std::cin >> i;

	//return 0;

	//std::cout << sys.GetStateVector() << std::endl;

	//std::cout << sys.GetVarianceMatrix() << std::endl;
	std::cout << "1\n";
	*/
	//youBot->SetValues(StatisticValue(4),SystemValueType::DISTURBANCE);

	
}