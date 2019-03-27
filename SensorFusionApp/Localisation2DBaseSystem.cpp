#include "pch.h"
#include "Localisation2DBaseSystem.h"

Localisation2DBaseSystem::Localisation2DBaseSystem(double Tdyn) :
	dTdyn(Tdyn) {}

/*
Eigen::VectorXd Localisation2DBaseSystem::Update(Eigen::VectorXd BaseSystemState,
	Eigen::VectorXd BaseSystemDisturbance, double Ts) const {
	Eigen::VectorXd newstates(getNumOfStates());
	Eigen::VectorXd newvelocities = ComputeNewLocalVelocities(BaseSystemState, BaseSystemDisturbance, Ts);
	// vx,vy,omega with a lowpass filter
	newstates(0) = BaseSystemState(0)*(1. - dTdyn / Ts) + newvelocities(0)*dTdyn / Ts;
	newstates(1) = BaseSystemState(1)*(1. - dTdyn / Ts) + newvelocities(1)*dTdyn / Ts;
	newstates(2) = BaseSystemState(2)*(1. - dTdyn / Ts) + newvelocities(2)*dTdyn / Ts;
	// x,y by transforming and integrating the local velocities
	newstates(3) = BaseSystemState(3) +
		(BaseSystemState(0)*cos(BaseSystemState(5)) + BaseSystemState(1)*sin(BaseSystemState(5)))*Ts;
	newstates(4) = BaseSystemState(4) +
		(-BaseSystemState(0)*sin(BaseSystemState(5)) + BaseSystemState(1)*cos(BaseSystemState(5)))*Ts;
	// phi by integrating the angular velocity
	newstates(5) = BaseSystemState(5) + BaseSystemState(2)*Ts;
	// additional states
	Eigen::VectorXd addStates = UpdateAdditionalStates(BaseSystemState, BaseSystemDisturbance, Ts);
	for (int i = 6; i < getNumOfStates(); i++)
		newstates(i) = addStates(i - 6);
	// return
	return newstates;
}*/

StatisticValue Localisation2DBaseSystem::getInitializationStates() const {
	return StatisticValue(Eigen::VectorXd::Zero(getNumOfStates()),
		Eigen::MatrixXd::Identity(getNumOfStates(), getNumOfStates()));
}



unsigned int Localisation2DBaseSystem::getNumOfStates() const { return 6+getNumOfAdditionalStates(); }
