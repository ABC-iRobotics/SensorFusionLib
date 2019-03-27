#pragma once

#include "BaseSystem.h"

// only dynamics
/*
	| vx    | = | 1-Td/Ts 0  0   0 0 0   | | vx     | + | Td/Ts 0 0 0 | | vx_new | + | 0              |
	| vy    |   | 0 1-Td/Ts  0   0 0 0   | | vy     |   | 0 Td/Ts 0 0 | | vy_new |   | 0              |
	| om    |   | 0  0  1-Td/Ts  0 0 0   | | om     |   | 0 0 Td/Ts 0 | | om_new |   | 0              |
	| x     |   | 0    0     0   1 0 0 0 | | x      |   | 0  0    0 0 | | dx_add |   | Ts*vx*cos(phi) |
	| y     |   | 0    0     0   0 1 0   | | y      |   | 0  0    0 0 |			     | Ts*vy*sin(phi) |
	| phi   |   | 0    0     Ts  0 0 1   | | phi    |   | 0  0    0 0 |			     | 0              |
	| x_add |   | -defined by A_add(Ts)- | | x_add  |   | 0  0    0 I |			     | 0              |
where
	| vx_new | = B_ * w  + f_(x,w)
	| vy_new |
	| om_new |
	| dx_add |
*/


class Localisation2DBaseSystem : public BaseSystem
{
private:
	const double dTdyn = 0.02;

public:
	Localisation2DBaseSystem(double Tdyn);

	virtual unsigned int getNumOfAdditionalStates() const = 0;

	virtual Eigen::MatrixXd getA_additional(double Ts) const = 0;

	Eigen::MatrixXd getA(double Ts) const override {
		Eigen::MatrixXd out0 = Eigen::MatrixXd::Identity(6, 6+getNumOfAdditionalStates());
		for (unsigned int i = 0; i < 3; i++)
			out0(i, i) = 1 - dTdyn / Ts;
		out0(5, 2) = Ts;
		if (getNumOfAdditionalStates() > 0) {
			Eigen::MatrixXd out = Eigen::MatrixXd(6 + getNumOfAdditionalStates(), 6 + getNumOfAdditionalStates());
			out << out0, getA_additional(Ts);
			return out;
		}
		else return out0;
	}

	// linear part of [vxnew, vynew, omeganew, dxadd]
	virtual Eigen::MatrixXd getB_(double Ts) const = 0;

	Eigen::MatrixXd getB(double Ts) const override {
		Eigen::MatrixXd out = Eigen::MatrixXd::Zero(6 + getNumOfAdditionalStates(), 3 + getNumOfAdditionalStates());
		for (unsigned int i = 0; i < 3; i++)
			out(i, i) = dTdyn / Ts;
		for (unsigned int i = 0; i < getNumOfAdditionalStates(); i++)
			out(6 + i, 3 + i) = 1;
		return out * getB_(Ts);
	}

	// nonlinear part of [vxnew, vynew, omeganew, dxadd]
	virtual Eigen::VectorXd UpdateNonlinearPart_(double Ts, Eigen::VectorXd BaseSystemState, Eigen::VectorXd BaseSystemDisturbance) const = 0;

	virtual Eigen::VectorXd UpdateNonlinearPart(double Ts, Eigen::VectorXd BaseSystemState, Eigen::VectorXd BaseSystemDisturbance) const {
		Eigen::MatrixXd out = Eigen::MatrixXd::Zero(6 + getNumOfAdditionalStates(), 3 + getNumOfAdditionalStates());
		for (unsigned int i = 0; i < 3; i++)
			out(i, i) = dTdyn / Ts;
		for (unsigned int i = 0; i < getNumOfAdditionalStates(); i++)
			out(6 + i, 3 + i) = 1;
		return out*UpdateNonlinearPart_(Ts, BaseSystemState, BaseSystemDisturbance);
	}

	// The nonlinear parts and the dependencies are by default zeros
	virtual Eigen::VectorXi getUpdateNonlinearXDependencies() const {
		Eigen::VectorXi out = Eigen::VectorXi::Zero(6 + getNumOfAdditionalStates());
		out(0) = 1; out(1) = 1; out(5) = 1;
		return out;
	}

	virtual Eigen::VectorXi getUpdateNonlinearWDependencies() const {
		return Eigen::VectorXi::Zero(getNumOfDisturbances());
	}
	/*
	Eigen::VectorXd Update(Eigen::VectorXd BaseSystemState, Eigen::VectorXd BaseSystemDisturbance, double Ts) const;

	virtual Eigen::VectorXd ComputeNewLocalVelocities(Eigen::VectorXd BaseSystemState,
		Eigen::VectorXd BaseSystemDisturbance, double Ts) const = 0;

	virtual Eigen::VectorXd UpdateAdditionalStates(Eigen::VectorXd BaseSystemState,
		Eigen::VectorXd BaseSystemDisturbance, double Ts) const = 0;
		*/
	virtual StatisticValue getInitializationStates() const override;

	unsigned int getNumOfStates() const override;

	//typedef std::shared_ptr<Localisation2DBaseSystem> Localisation2DBaseSystemPtr;
};
