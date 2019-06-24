#include <iostream>

#include "Simulation_youbot_WAUKF.h"
#include "WAUKF.h"
#include "youBotSystem.h"
#include "INSSensor.h"
#include "AbsoluthePoseSensor.h"

#ifdef FILTERPLOT
#include "FilterPlot.h"
#endif

void Simulation_youBot_WAUKF() {
	// Trajectory
	Trajectory traj = genTrajectory();

	// Virtual sensors
	INS insphantom(0.1, 0.1, 0.1);
	youBotKinematics youbotphantom(0.4, 0.25, 0.05, 0.01);
	double GPSnoise = 0.01;
	AbsSensor GPS(GPSnoise);

	// Init sensor system
	WAUKF::WAUKFPtr filter;
	//youbot
	BaseSystem::BaseSystemPtr youBot = std::make_shared<youBotSystem>(0.1, 0.4, 0.25, 0.05);
	youBot->systemTest();
	{
		// Init state (vx,vy,om,x,y,phi,null)
		StatisticValue initState = StatisticValue(Eigen::VectorXd::Zero(7), Eigen::MatrixXd::Identity(7, 7)*1.1);
		// Init noise ()
		StatisticValue initNoise(0);
		// Init disturbance (FL,FR,BL,BR) : must be updated later
		StatisticValue initDist(Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Identity(4, 4)*0.01);
		// Measurement: always zero
		Eigen::VectorXd constantMeasurement = Eigen::VectorXd::Zero(1);
		SystemManager::BaseSystemData data2 = SystemManager::BaseSystemData(youBot,
			initNoise, initDist, constantMeasurement, SystemManager::MeasurementStatus::CONSTANT);
		filter = std::make_shared<WAUKF>(data2, initState);
	}

	//ins
	Sensor::SensorPtr ins = std::make_shared<INSSensor>(youBot);
	ins->systemTest();
	{
		// Init state (xs,ys,dphi)
		Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
		x0[2] = 0.2;
		StatisticValue initState = StatisticValue(x0, Eigen::MatrixXd::Identity(3, 3)*0.3);
		// Init noise ()
		StatisticValue initNoise(0);
		// Init disturbance () 
		StatisticValue initDist(Eigen::VectorXd::Ones(3)*0, Eigen::MatrixXd::Identity(3, 3)*0.5);
		SystemManager::SensorData data = SystemManager::SensorData(ins, initNoise, initDist);
		filter->AddSensor(data, initState);
		filter->SetDisturbanceValueWindowing(ins, 100);
		filter->SetDisturbanceVarianceWindowing(ins, 100);
		
	}

	Sensor::SensorPtr absPose = std::make_shared<AbsoluthePoseSensor>(youBot, true);
	absPose->systemTest();
	{
		// Init state (x,y,phi)
		StatisticValue initState = StatisticValue(0);
		// Init noise (vx,vy,vphi)
		StatisticValue initNoise(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)*GPSnoise*3);
		// Init disturbance () 
		StatisticValue initDist(0);
		SystemManager::SensorData data = SystemManager::SensorData(absPose, initNoise, initDist);
		filter->AddSensor(data, initState);
		filter->SetNoiseVarianceWindowing(absPose, 100);
		//filter->SetNoiseValueWindowing(absPose, 100);
	}

#ifdef FILTERPLOT
	FilterPlot plotter(*filter, youBot, STATE);
	//FilterPlot plotter1(*filter, ins, STATE);
	FilterPlot plotter2(*filter, absPose, OUTPUT);
	FilterPlot plotter3(*filter, ins, OUTPUT);
#endif
	// Simulation
	for (size_t n = 0; n < traj.length(); n++) {
		youBot->SetDisturbanceValue(youbotphantom.update(traj.vx_local[n], traj.vy_local[n], traj.omega[n]));

		if (n % 3 == 0)
			absPose->MeasurementDone(GPS.update(traj.x[n], traj.y[n], traj.phi[n]));

		insphantom.Step(traj.ax_local[n], traj.ay_local[n], traj.omega[n], traj.Ts);
		ins->MeasurementDone(insphantom.Out());

		//std::cout << (*filter)(STATE).vector(9) << std::endl;


		Eigen::VectorXd truth(7);
		truth(0) = traj.vx_local[n];
		truth(1) = traj.vy_local[n];
		truth(2) = traj.omega[n];
		truth(3) = traj.x[n];
		truth(4) = traj.y[n];
		truth(5) = traj.phi[n];
		truth(6) = 0;
		plotter.Callback(FilterCallData(truth, youBot, n*traj.Ts, STATE, FilterCallData::GROUNDTRUTH));




		filter->Step(traj.Ts);
#ifdef FILTERPLOT
		if (n % 200 == 0)
			FilterPlot::UpdateWindows();
#endif
	}

	std::cout << "Ready\n";
	unsigned int in;
	std::cin >> in;

}
