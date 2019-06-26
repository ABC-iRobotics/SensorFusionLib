// SensorFusion.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"

// Fasten the system:
// partitionate matrices - helper quantities
// evalnonlinpart on matrices

#include "Simulation_youbot_Kalman.h"
#include "Simulation_youbot_WAUKF.h"

int main() {

	simulation_youbot_Kalman();

	//Simulation_youBot_WAUKF();

	return 0;

}