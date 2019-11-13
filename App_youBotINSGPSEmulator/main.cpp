#include "msg2buf.h"
#include <thread>
#include "ZMQPublisher.h"
#include "Simulation.h"

int main() {
	ZMQPublisher pub_odometry("tcp://*:5555");
	ZMQPublisher pub_IMU("tcp://*:5556");
	ZMQPublisher pub_GPS("tcp://*:5557");

	// Trajectory
	Trajectory traj = genTrajectory();

	// Virtual sensors
	INS insphantom(0.1, 0.1, 0.1);
	youBotKinematics youbotphantom(0.4, 0.25, 0.05, 0.01);
	double GPSnoise = 0.01;
	AbsSensor GPS(GPSnoise);

	//Simulate youBot and sensors, send data
	TimeMicroSec t0;
	// Simulation
	for (size_t n = 0; n < traj.length(); n++) {
		TimeMicroSec t = t0 + traj.Ts * n;
		if (t >= TimeMicroSec())
			(t - TimeMicroSec()).Sleep_for();
		else printf("-");
		{
			Eigen::VectorXd w_youbot = youbotphantom.update(traj.vx_local[n], traj.vy_local[n], traj.omega[n]);
			DataMsg msg(0, DISTURBANCE, SENSOR);
			msg.SetValueVector(w_youbot);
			pub_odometry.SendMsg(msg);
		}
		if (n % 30 == 0) {
			Eigen::VectorXd y_GPS = GPS.update(traj.x[n], traj.y[n], traj.phi[n]);
			DataMsg msg(1, OUTPUT, SENSOR);
			msg.SetValueVector(y_GPS);
			pub_IMU.SendMsg(msg);
		}
		insphantom.Step(traj.ax_local[n], traj.ay_local[n], traj.omega[n], traj.Ts.TimeInS());
		{
			Eigen::VectorXd y_ins = insphantom.Out();
			DataMsg msg(2, OUTPUT, SENSOR);
			msg.SetValueVector(y_ins);
			pub_GPS.SendMsg(msg);
		}
	}
	return 0;
}