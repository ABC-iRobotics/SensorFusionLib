#include <iostream>

#include "Simulation_youbot_Kalman.h"
#include "KalmanFilter.h"
#include "KUKAyouBot.h"
#include "INSSensor.h"
#include "AbsoluthePoseSensor.h"


#ifdef FILTERPLOT
#include "FilterPlot.h"
#endif

void simulation_youbot_Kalman() {
	// Trajectory
	Trajectory traj = genTrajectory();

	// Virtual sensors
	INS insphantom(0.1, 0.1, 0.1);
	youBotKinematics youbotphantom(0.4, 0.25, 0.05, 0.01);
	double GPSnoise = 0.01;
	AbsSensor GPS(GPSnoise);

	// Init sensor system
	KalmanFilter::KalmanFilterPtr filter;
	//youbot
	BaseSystem::BaseSystemPtr youBot = std::make_shared<KUKAyouBot>(0.1, 0.4, 0.25, 0.05, 0);
	youBot->systemTest();
	{
		// Init state (vx,vy,om,x,y,phi,null)
		StatisticValue initState = StatisticValue(Eigen::VectorXd::Zero(7), Eigen::MatrixXd::Identity(7, 7)*1);
		// Init noise ()
		StatisticValue initNoise(0);
		// Init disturbance (FL,FR,BL,BR) : must be updated later
		StatisticValue initDist(Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Identity(4, 4)*0.01);
		// Measurement: always zero
		Eigen::VectorXd constantMeasurement = Eigen::VectorXd::Zero(1);
		SystemManager::BaseSystemData data2 = SystemManager::BaseSystemData(youBot,
			initNoise, initDist, constantMeasurement, SystemManager::MeasurementStatus::CONSTANT);
		filter = std::make_shared<KalmanFilter>(data2, initState);
	}

	//ins
	Sensor::SensorPtr ins = std::make_shared<INSSensor>(youBot, 2);
	ins->systemTest();
	{
		// Init state (xs,ys,dphi)
		Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
		x0[2] = 0.2;
		StatisticValue initState = StatisticValue(x0, Eigen::MatrixXd::Identity(3, 3)*0.3);
		// Init noise (vx,vy,v_om)
		StatisticValue initNoise(0);// Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)*0.1);
		// Init disturbance () 
		StatisticValue initDist(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)*0.05);
		SystemManager::SensorData data = SystemManager::SensorData(ins, initNoise, initDist);
		filter->AddSensor(data, initState);
	}

	Sensor::SensorPtr absPose = std::make_shared<AbsoluthePoseSensor>(youBot, 1, true);
	absPose->systemTest();
	{
		// Init state (x,y,phi)
		StatisticValue initState = StatisticValue(0);
		// Init noise (vx,vy,vphi)
		StatisticValue initNoise(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)*GPSnoise);
		// Init disturbance () 
		StatisticValue initDist(0);
		SystemManager::SensorData data = SystemManager::SensorData(absPose, initNoise, initDist);
		filter->AddSensor(data, initState);
	}
#ifdef FILTERPLOT
	filter->SetCallback(FilterPlot::AddDataToWindows);
	FilterPlot plotter(youBot->getID(), youBot->getName(), youBot->getStateNames(), STATE);
	//FilterPlot plotter2(youBot->getID(), youBot->getName(), youBot->getDisturbanceNames(), DISTURBANCE);
	//FilterPlot plotter3(ins->getID(), ins->getName(), ins->getOutputNames(), OUTPUT);
#endif
	// Simulation
	for (size_t n = 0; n < traj.length(); n++) {
		DataMsg data(youBot->getID(), DISTURBANCE, SENSOR, traj.Ts*n);
		data.SetValueVector(youbotphantom.update(traj.vx_local[n], traj.vy_local[n], traj.omega[n]));
		filter->SetProperty(data);

		if (n % 30 == 0)
		{
			data = DataMsg(absPose->getID(), OUTPUT, SENSOR, traj.Ts*n);
			data.SetValueVector(GPS.update(traj.x[n], traj.y[n], traj.phi[n]));
			filter->SetProperty(data);
		}

		insphantom.Step(traj.ax_local[n], traj.ay_local[n], traj.omega[n], traj.Ts.TimeInS());
		data = DataMsg(ins->getID(), OUTPUT, SENSOR, traj.Ts*n);
		data.SetValueVector(insphantom.Out());
		filter->SetProperty(data);

		Eigen::VectorXd truth(7);
		truth(0) = traj.vx_local[n];
		truth(1) = traj.vy_local[n];
		truth(2) = traj.omega[n];
		truth(3) = traj.x[n];
		truth(4) = traj.y[n];
		truth(5) = traj.phi[n];
		truth(6) = 0;
		{
			DataMsg data(youBot->getID(), STATE, GROUND_TRUTH, traj.Ts*n); //a bit ugly in the plot because of different timestamps...
			data.SetValueVector(truth);
#ifdef FILTERPLOT
			plotter.AddDataToThis(data);
#endif
		}

		filter->Step(traj.Ts);
#ifdef FILTERPLOT
		if (n % 2 == 0)
			FilterPlot::UpdateWindows();
#endif
	}

	std::cout << "Ready\n";
	unsigned int in;
	std::cin >> in;

}
