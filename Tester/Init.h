#pragma once

#include "FilterSystem.h"
#include "youBotSystem.h"
#include "IMUsensor.h"

#include "SystemManager.h"

FilterSystem::SystemData InitYouBotSystem(BaseSystem::BaseSystemPtr ptr, StatisticValue& initState) {
	// Init state (vx,vy,om,x,y,phi,null)
	initState = StatisticValue(Eigen::VectorXd::Zero(7), Eigen::MatrixXd::Identity(7, 7)*1.1);
	// Init noise ()
	StatisticValue initNoise(0);
	// Init disturbance (FL,FR,BL,BR) : must be updated later
	StatisticValue initDist(Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Identity(4, 4)*0.01);
	// Measurement: always zero
	Eigen::VectorXd constantMeasurement = Eigen::VectorXd::Zero(1);
	return FilterSystem::SystemData(ptr, initNoise, initDist, constantMeasurement, FilterSystem::CONSTANT);
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
	return SystemManager::BaseSystemData(ptr, initNoise, initDist, constantMeasurement, SystemManager::CONSTANT);
}

FilterSystem::SystemData InitIMUSensor(Sensor::SensorPtr ptr, StatisticValue& initState) {
	// Init state (vx_old,vy_old,sx,sy,som)
	initState = StatisticValue(Eigen::VectorXd::Zero(2), Eigen::MatrixXd::Identity(2, 2)*2.);
	initState.Add(StatisticValue(Eigen::VectorXd::Ones(3), Eigen::MatrixXd::Identity(3, 3)*0.1));
	// Init noise (bx,by,bom)
	StatisticValue initNoise(Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)*1.1);
	// Init disturbance () 
	StatisticValue initDist(0);
	return FilterSystem::SystemData(ptr, initNoise, initDist);
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