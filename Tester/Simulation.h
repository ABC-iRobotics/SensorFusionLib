#pragma once
#include <random>
#include "Eigen/Dense"


struct Random {
	static std::default_random_engine generator;
	static std::normal_distribution<double> dist;
	static double scalar();
	Eigen::MatrixXd sqrtS;
	Random(Eigen::MatrixXd S);
	Random(int N, double s) : sqrtS(Eigen::MatrixXd::Identity(N, N)*sqrt(s)) {}
	Eigen::VectorXd get();
};

typedef std::vector<double> Values;
double last_element(const Values& v);

struct Trajectory {
	double Ts;
	Values x, y, phi;
	Values vx_local, vy_local, omega;
	Values ax_local, ay_local, epsilon;

	void add(double ax, double ay, double eps);
	size_t length() const;
	Trajectory(double Ts_);
};

Trajectory genTrajectory();

struct INS {
	double vx, vy, x, y, phi;
	double sqrtsax, sqrtsay, sqrtsom;
	void Step(double ax, double ay, double om, double Ts);
	INS(double sax, double say, double som);
	Eigen::VectorXd Out() const;
};

struct youBotKinematics {
	double L, R, W;
	Eigen::MatrixXd M;
	Random disturbance;
	youBotKinematics(double L_, double W_, double R_, double s);
	Eigen::VectorXd update(double vx, double vy, double omega);
};

struct AbsSensor {
	Eigen::MatrixXd M;
	Random disturbance;
	AbsSensor(double s) : disturbance(3,s) {};
	Eigen::VectorXd update(double x, double y, double phi);;
};