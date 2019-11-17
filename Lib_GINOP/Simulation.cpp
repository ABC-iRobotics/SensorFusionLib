#include "Simulation.h"

Random::Random(Eigen::MatrixXd S) {
	Eigen::LLT<Eigen::MatrixXd> lltOfA(S); // compute the Cholesky decomposition of A
	sqrtS = lltOfA.matrixL(); // retrieve factor L  in the decomposition
}

double Random::scalar() {
	return dist(generator);
}

Eigen::VectorXd Random::get() {
	Eigen::VectorXd out = Eigen::VectorXd(sqrtS.cols());
	for (unsigned int i = 0; i < sqrtS.cols(); i++)
		out(i) = dist(generator);
	return sqrtS * out;
}

std::normal_distribution<double> Random::dist = std::normal_distribution<double>(0, 1);
std::default_random_engine Random::generator = std::default_random_engine();

double last_element(const Values& v) {
	if (v.size() > 0)
		return v[v.size() - 1];
	return 0;
}

void Trajectory::add(double ax, double ay, double eps) {
	ax_local.push_back(ax);
	ay_local.push_back(ay);
	epsilon.push_back(eps);

	double vx = last_element(vx_local);
	double vy = last_element(vy_local);
	double om = last_element(omega);
	double ph = last_element(phi);

	vx_local.push_back(vx + Ts.TimeInS() * (ax + om * vy));
	vy_local.push_back(vy + Ts.TimeInS() * (ay - om * vx));
	omega.push_back(om + Ts.TimeInS() * eps);

	x.push_back(last_element(x) + Ts.TimeInS() * (vx * cos(ph) - vy * sin(ph)));
	y.push_back(last_element(y) + Ts.TimeInS() * (vy * cos(ph) + vx * sin(ph)));
	phi.push_back(last_element(phi) + om * Ts.TimeInS() + eps * Ts.TimeInS()*Ts.TimeInS() / 2);
}

size_t Trajectory::length() const {
	return x.size();
}

Trajectory::Trajectory(TimeMicroSec Ts_) : Ts(Ts_) {
	x = Values(); y = Values(); phi = Values();
	vx_local = Values(); vy_local = Values(); omega = Values();
	ax_local = Values(); ay_local = Values(); epsilon = Values();
}

Trajectory genTrajectory() {
	double Ts = 0.01;
	Trajectory traj(TimeMicroSec(Ts*1e6));
	for (unsigned int n = 0; n < (0.5 / Ts); n++)
		traj.add(0, 0, 0);
	for (unsigned int n = 0; n < (1. / Ts); n++)
		traj.add(3., 0, 0);
	for (unsigned int n = 0; n < (0.1 / Ts); n++) {
		double vx = last_element(traj.vx_local);
		double vy = last_element(traj.vy_local);
		double om = last_element(traj.omega);
		traj.add(0, (10*vx-vy/Ts)*0.5, 10);
	}
		
	for (unsigned int n = 0; n < (10. / Ts); n++)
		traj.add(0, last_element(traj.omega) * last_element(traj.vx_local), 0);
	return traj;
}

void INS::Step(double ax, double ay, double om, double Ts) {
	double ax_ = ax + sqrtsax * Random::scalar();
	double ay_ = ay + sqrtsay * Random::scalar();
	double om_ = om + sqrtsom * Random::scalar();
	x += Ts * (vx * cos(phi) - vy * sin(phi));
	y += Ts * (vy * cos(phi) + vx * sin(phi));
	phi += om * Ts;
	vx += Ts * (ax_ + om_ * vy);
	vy += Ts * (ay_ - om_ * vx);
}

INS::INS(double sax, double say, double som) : vx(0), vy(0), x(0), y(0), phi(0.2),
	sqrtsax(sqrt(sax)), sqrtsay(sqrt(say)), sqrtsom(sqrt(som)) {}

Eigen::VectorXd INS::Out() const {
	Eigen::VectorXd out(3);
	out[0] = x;
	out[1] = y;
	out[2] = phi;
	return out;
}

youBotKinematics::youBotKinematics(double L_, double W_, double R_, double s) : disturbance(4, s),
L(L_), R(R_), W(W_) {
	M = Eigen::MatrixXd::Ones(4, 3);
	M(0, 1) = -1;
	M(0, 2) = -1;
	M(2, 2) = -1;
	M(3, 1) = -1;
}

Eigen::VectorXd youBotKinematics::update(double vx, double vy, double omega) {
	Eigen::VectorXd v(3);
	v(0) = vx / R;
	v(1) = vy / R;
	v(2) = omega / 2. / R * (W + L);
	return M * v + disturbance.get();
}

Eigen::VectorXd AbsSensor::update(double x, double y, double phi) {
	Eigen::VectorXd out(3);
	out[0] = x; out[1] = y; out[2] = phi;
	return out + disturbance.get();
}
