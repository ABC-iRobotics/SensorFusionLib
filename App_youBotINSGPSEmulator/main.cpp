#include "msg2buf.h"
#include <thread>
#include "ZMQPublisher.h"
#include "Simulation.h"

int main() {
	ZMQPublisher pub(5555);

	// Trajectory
	Trajectory traj = genTrajectory();

	// Virtual sensors
	INS insphantom(0.1, 0.1, 0.1);
	youBotKinematics youbotphantom(0.4, 0.25, 0.05, 0.01);
	double GPSnoise = 0.01;
	AbsSensor GPS(GPSnoise);

	//Simulate youBot and sensors, send data
	unsigned long t0_ms = getTimeInMicroseconds();
	// Simulation
	for (size_t n = 0; n < traj.length(); n++) {
		unsigned long t_ms = t0_ms + n * traj.Ts*1e6;
		if (t_ms >= getTimeInMicroseconds())
			std::this_thread::sleep_for(std::chrono::duration<unsigned long, std::micro>(t_ms - getTimeInMicroseconds()));
		else printf("-");
		{
			Eigen::VectorXd w_youbot = youbotphantom.update(traj.vx_local[n], traj.vy_local[n], traj.omega[n]);
			SystemDataMsg msg(0, SystemDataMsg::TOFILTER_DISTURBANCE, getTimeInMicroseconds());
			msg.SetValueVector(w_youbot);
			pub.SendMsg(msg);
		}
		if (n % 30 == 0) {
			Eigen::VectorXd y_GPS = GPS.update(traj.x[n], traj.y[n], traj.phi[n]);
			SystemDataMsg msg(1, SystemDataMsg::TOFILTER_MEASUREMENT, getTimeInMicroseconds());
			msg.SetValueVector(y_GPS);
			pub.SendMsg(msg);
		}
		insphantom.Step(traj.ax_local[n], traj.ay_local[n], traj.omega[n], traj.Ts);
		{
			Eigen::VectorXd y_ins = insphantom.Out();
			SystemDataMsg msg(2, SystemDataMsg::TOFILTER_MEASUREMENT, getTimeInMicroseconds());
			msg.SetValueVector(y_ins);
			pub.SendMsg(msg);
		}
	}
	return 0;
}