#pragma once
// Does not perform any computation
// Only describes the base system (linear,angular velocity, orientation, position)
//
// The dynamics and output of the base system can depend on its own states and
// the corresponding  disturbances and measurement noises

#include "System.h"

class BaseSystem : public System {
public:
	BaseSystem() {};

	// The dynamics can be defined by the linear coefficient matrices and the nonlinearpart
	// x(k) = A * x(k-1) + B * w(k) + UpdateNonlinPart(x(k-1), w(k)));
	// y(k) = C * x(k)   + D * v(k) + OutputNonlinearPart(x(k), v(k));
	// but the dependencies of the nonlinearparts  on the x,w,v vectors must be elementwisely given
	virtual Eigen::MatrixXd getA(double Ts) const = 0;

	virtual Eigen::MatrixXd getB(double Ts) const = 0;
	
	virtual Eigen::MatrixXd getC(double Ts) const = 0;
	
	virtual Eigen::MatrixXd getD(double Ts) const = 0;


	// The nonlinear parts and the dependencies are by default zeros
	virtual Eigen::VectorXi getUpdateNonlinearXDependencies() const;

	virtual Eigen::VectorXi getUpdateNonlinearWDependencies() const;

	virtual Eigen::VectorXi getOutputNonlinearXDependencies() const;

	virtual Eigen::VectorXi getOutputNonlinearVDependencies() const;

	virtual Eigen::VectorXd UpdateNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& disturbance) const;

	virtual Eigen::VectorXd OutputNonlinearPart(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& noise) const;
	

	// The Eval functions execute the prediction/output computation with the given values and defined coefficients/functions
	Eigen::VectorXd EvalUpdate(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& BaseSystemDisturbance) const;

	Eigen::VectorXd EvalOutput(double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& BaseSystemNoise) const;

	Eigen::VectorXd genNonlinearPart(UpdateType type, double Ts, const Eigen::VectorXd& state, const Eigen::VectorXd& in) const;

	Eigen::VectorXi genNonlinearDependency(UpdateType outType, InputType inType);

	typedef std::shared_ptr<BaseSystem> BaseSystemPtr;

	void systemTest() const;
};