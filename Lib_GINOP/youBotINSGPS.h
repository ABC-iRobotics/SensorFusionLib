#pragma once
#include "FilteringManager.h"
#include "WAUKF.h"
#include "youBotSystem.h"
#include "INSSensor.h"
#include "AbsoluthePoseSensor.h"

class youBotINSGPS : public FilteringManager {

public:
	youBotINSGPS(int port, int port_logger=-1) : FilteringManager(0.01, port) {
		WAUKF::WAUKFPtr waukf;

		//youbot
		BaseSystem::BaseSystemPtr youBot = std::make_shared<youBotSystem>(0.1, 0.4, 0.25, 0.05, 0);
		{
			youBot->systemTest();
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
			waukf = std::make_shared<WAUKF>(data2, initState);
			_addSystem(youBot);
		}

		//ins
		{
			Sensor::SensorPtr ins = std::make_shared<INSSensor>(youBot,2);
			ins->systemTest();
			// Init state (xs,ys,dphi)
			Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
			x0[2] = 0.2;
			StatisticValue initState = StatisticValue(x0, Eigen::MatrixXd::Identity(3, 3)*0.3);
			// Init noise ()
			StatisticValue initNoise(0);
			// Init disturbance () 
			StatisticValue initDist(Eigen::VectorXd::Ones(3) * 0, Eigen::MatrixXd::Identity(3, 3)*0.5);
			SystemManager::SensorData data = SystemManager::SensorData(ins, initNoise, initDist);
			waukf->AddSensor(data, initState);
			waukf->SetDisturbanceValueWindowing(ins, 100);
			waukf->SetDisturbanceVarianceWindowing(ins, 100);
			_addSystem(ins);
		}
		// GPS
		{
			Sensor::SensorPtr absPose = std::make_shared<AbsoluthePoseSensor>(youBot, 1, true);
			absPose->systemTest();
			// Init state (x,y,phi)
			StatisticValue initState = StatisticValue(0);
			// Init noise (vx,vy,vphi)
			double GPSnoise = 0.01;
			StatisticValue initNoise(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)*GPSnoise * 3);
			// Init disturbance () 
			StatisticValue initDist(0);
			SystemManager::SensorData data = SystemManager::SensorData(absPose, initNoise, initDist);
			waukf->AddSensor(data, initState);
			waukf->SetNoiseVarianceWindowing(absPose, 100);
			//waukf->SetNoiseValueWindowing(absPose, 100);
			_addSystem(absPose);
		}
		filter = waukf;
		if (port_logger >=0)
			SetZMQLogger(port_logger);
	}

	~youBotINSGPS();
};

