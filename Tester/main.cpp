// SensorFusion.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <thread>
#include "FilterPlot.h"
#include "KalmanFilter.h"
#include "youBotSystem.h"
#include "IMUSensor.h"


void FilterCallback(FilterCallData data) {
	if (data.callType == FilterCallData::FILTERING && data.type == STATE) {
		std::cout << data.value.vector.transpose() << std::endl;
	}
}

SystemManager::BaseSystemData InitYouBotSystem2(BaseSystem::BaseSystemPtr ptr, StatisticValue& initState) {
	// Init state (vx,vy,om,x,y,phi,null)
	initState = StatisticValue(Eigen::VectorXd::Zero(7), Eigen::MatrixXd::Identity(7, 7)*1.1);
	// Init noise ()
	StatisticValue initNoise(0);
	// Init disturbance (FL,FR,BL,BR) : must be updated later
	StatisticValue initDist(Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Identity(4, 4)*0.01);
	// Measurement: always zero
	Eigen::VectorXd constantMeasurement = Eigen::VectorXd::Zero(1);
	return SystemManager::BaseSystemData(ptr, initNoise, initDist, constantMeasurement, SystemManager::MeasurementStatus::CONSTANT);
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

// Fasten the system:
// partitionate matrices - helper quantities
// evalnonlinpart on matrices

#include <ctime>

#include "WAUKF.h"

#include "Simulation_youbot_Kalman.h"

int main() {
	simulation_youbot_Kalman();

	

	return 0;

	BaseSystem::BaseSystemPtr youBot = std::make_shared<youBotSystem>(0.2, 0.4, 0.25, 0.05);
	StatisticValue initState;
	SystemManager::BaseSystemData data2 = InitYouBotSystem2(youBot, initState);
	KalmanFilter filter(data2, initState);

	Windowing::SystemEstimationOptions baseSystemOptions(Windowing::WindowOptions<Eigen::VectorXd>(100, Eigen::VectorXd::Zero(4)),
		Windowing::WindowOptions<Eigen::MatrixXd>(), Windowing::WindowOptions<Eigen::VectorXd>(), Windowing::WindowOptions<Eigen::MatrixXd>());

	Sensor::SensorPtr imu = std::make_shared<IMUSensor>(youBot);
	SystemManager::SensorData data3 = InitIMUSensor2(imu, initState);


	Windowing::WAUKF f(data2, initState, baseSystemOptions);
	f.AddSensor(data3, initState, baseSystemOptions);


	auto p = f.getPartitioner();

	Eigen::MatrixXd S = Eigen::MatrixXd::Identity(12, 12);

	p.PartVariance(STATE, S, 0, 0)(1, 0) = 2;

	std::cout << p.PartVariance(STATE, S, -1, -1) << std::endl << std::endl;
	std::cout << p.PartVariance(STATE, S, 0, -1) << std::endl << std::endl;
	std::cout << p.PartVariance(STATE, S, -1, 0) << std::endl << std::endl;
	std::cout << p.PartVariance(STATE, S, 0, 0) << std::endl << std::endl;

	Eigen::VectorXd x = Eigen::VectorXd::Ones(12);
	x(7) = 5;
	std::cout << p.PartValue(STATE, x, -1) << std::endl << std::endl;
	std::cout <<  p.PartValue(STATE,x,0) << std::endl << std::endl;



	
	filter.AddSensor(data3, initState);

	//filter.AddCallback(FilterCallback);
	FilterPlot plotter(filter, imu, OUTPUT);
	FilterPlot plotter2(filter, imu, STATE);
	FilterPlot plotter3(filter, youBot, STATE);
	FilterPlot plotter4(filter, imu, NOISE);
	
	Eigen::VectorXd a = Eigen::VectorXd(3);
	a(0) = 0.1; a(1) = 0.3; a(2) = 0.4;

	std::clock_t c_start = std::clock();
	for (unsigned int i=0; i<1000; i++) {
		imu->MeasurementDone(a);
		filter.Step(0.001);
		//std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10.));
		//FilterPlot::UpdateWindows();
		std::cout << filter(STATE).variance(0, 0) << std::endl;
	}
	std::clock_t c_end = std::clock();
	long double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
	std::cout << "CPU time used: " << time_elapsed_ms << " ms\n";


	return 0;
	/*
	std::cout << man.EvalNonLinPart(0.001, System::TIMEUPDATE, man(STATE).vector, man(DISTURBANCE).vector) << std::endl << std::endl;
	std::cout << man.EvalNonLinPart(0.001, System::MEASUREMENTUPDATE, man(STATE).vector, man(NOISE).vector) << std::endl << std::endl;

	std::cout << man.num(DISTURBANCE) << std::endl;
	std::cout << man.num(STATE) << std::endl;
	std::cout << man.num(NOISE) << std::endl;
	std::cout << man.num(OUTPUT) << std::endl;

	SystemValueType type = OUTPUT;
	std::cout << man(type).vector << std::endl << std::endl;
	std::vector<Eigen::VectorXd> temp = man.partitionate(type, man(type).vector);
	for (int i = 0; i < temp.size(); i++)
		std::cout << temp[i] << std::endl << std::endl;

	std::cout << man(STATE) << std::endl << std::endl;

	std::cout << A << std::endl << std::endl;
	std::cout << B << std::endl << std::endl;

	return 0;
	*/
}