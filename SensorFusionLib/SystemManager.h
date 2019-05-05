#pragma once

#include "Sensor.h"

#include "PartialCholevski.h"


/* Only to store/describe the system and provide an interface to their interface -> it does not perform filtering
*/
class SystemManager
{
public:
	enum MeasurementStatus { OBSOLETHE, UPTODATE, CONSTANT };

	class SystemData {
		MeasurementStatus measStatus;
		StatisticValue noise;
		StatisticValue disturbance;
		Eigen::VectorXd measurement;
	public:
		SystemData(StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_ = Eigen::VectorXd(), MeasurementStatus measStatus_ = OBSOLETHE);
		virtual System::SystemPtr getPtr() const = 0;
		StatisticValue operator()(SystemValueType type) const; // returns th given value
		size_t num(SystemValueType type) const; // return length of the given value accroding to the measStatus
		size_t num0(SystemValueType type) const; // return length of the given value
		void set(StatisticValue value, SystemValueType type); // set the given value
		void resetMeasurement();
		bool available() const; // returns if is measurement available
		virtual bool isBaseSystem() const = 0;
	};

	class BaseSystemData : public SystemData {
		BaseSystem::BaseSystemPtr ptr;
	public:
		BaseSystemData(BaseSystem::BaseSystemPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_ = Eigen::VectorXd(), MeasurementStatus measStatus_ = OBSOLETHE);
		Eigen::VectorXi dep(System::UpdateType outType, System::InputType type) const;
		Eigen::MatrixXd getMatrix(double Ts, System::UpdateType type, System::InputType inType) const;
		BaseSystem::BaseSystemPtr getBaseSystemPtr() const;
		System::SystemPtr getPtr() const override;
		bool isBaseSystem() const override;
	};

	class SensorData : public SystemData {
		Sensor::SensorPtr ptr;
	public:
		SensorData(Sensor::SensorPtr ptr_, StatisticValue noise_, StatisticValue disturbance_,
			Eigen::VectorXd measurement_ = Eigen::VectorXd(), MeasurementStatus measStatus_ = OBSOLETHE);
		Eigen::VectorXi depSensor(System::UpdateType outType, System::InputType type) const;
		Eigen::VectorXi depBaseSystem(System::UpdateType outType, System::InputType type) const;
		Eigen::MatrixXd getMatrixBaseSystem(double Ts, System::UpdateType type, System::InputType inType) const;
		Eigen::MatrixXd getMatrixSensor(double Ts, System::UpdateType type, System::InputType inType) const;
		Sensor::SensorPtr getSensorPtr() const;
		System::SystemPtr getPtr() const override;;
		bool isBaseSystem() const override;
	};

private:
	unsigned int ID; // unique ID for identifying callbacks

	typedef std::vector<SensorData> SensorList;
	SensorList sensorList;
	BaseSystemData baseSystem;

	StatisticValue state;
//protected:
public:

	int _GetIndex(unsigned int ID) const;

	size_t nSensors() const;

	size_t num(SystemValueType type) const;

	SensorData Sensor(size_t index) const;
	SensorData & Sensor(size_t index);

	const SystemData* SystemByID(unsigned int ID) const;
	SystemData* SystemByID(unsigned int ID);

	// Get the whole STATE, DISTURBANCE values as a StatisticValue instance
	// OR
	// get the measured OUTPUT for the available (=not obsolethe) systems (sensors & basesystem)
	// OR
	// get the noises for the basesystem and the active sensors
	StatisticValue operator()(SystemValueType type) const;

	/* Signals:
	   x = [ x_bs' x_s1' ... x_sn' ]'
	   w = [ w_bs' w_s1' ... w_sn' ]'
	   y = [ y_bs' y_s1' ... y_sn' ]' y_i = [] if there is not uptodate measurement
	   v = [ v_bs' v_s1' ... v_sn' ]' v_si = [] if there is not updtodate measurement
	   Update model:
	   x = A*x + f(x,w) + B*w
	   Output model:
	   y = C*x + g(x,v) + D*v;
	*/

	// Partitionate back the STATE, DISTURBANCE vectors
	// OR
	// the measured OUTPUT for the available (=not obsolethe) systems (sensors & basesystem)
	// OR
	// the noises for the basesystem and the active sensors
	std::vector<Eigen::VectorXd> partitionate(SystemValueType type, Eigen::VectorXd value) const;

	/* Get A,B, C,D matrices according to the available sensors*/
	void getMatrices(System::UpdateType out_, double Ts, Eigen::MatrixXd& A, Eigen::MatrixXd& B) const;

	// could be faster....
	Eigen::VectorXd EvalNonLinPart(double Ts, System::UpdateType outType, Eigen::VectorXd state, Eigen::VectorXd in) const;

	/*Eigen::MatrixXd GetSelector_(const Eigen::VectorXi& nonlinDep, bool nonlin) {
		Eigen::Index outRows = nonlinDep.sum();
		if (!nonlin) outRows = nonlinDep.size() - outRows;
		Eigen::MatrixXd out = Eigen::MatrixXd::Zero(outRows, nonlinDep.size());
		unsigned int i = 0;
		for (unsigned int n = 0; n < nonlinDep.size(); n++)
			if ((nonlinDep[n] == 0) ^ nonlin) {
				out(i, n) = 1;
				i++;
			}
		return out;
	}*/

	StatisticValue Eval(System::UpdateType outType, StatisticValue state, StatisticValue in, Eigen::MatrixXd& S_out_, Eigen::MatrixXd S_out_in) {
		// Concatenate dep vectors

		// Apply partial chol on the considered columns

		// 

		// Get selector matrices ?

		// 
		/*
		int nNL_state = 0, nNL_in = 0;
		Eigen::VectorXi = systemList
		for (size_t i = 1; i < nSystems(); i++)
			if (outType == System::UPDATE || (outType == System::OUT && systemList[i].available())) {
			nNL_state
		}


		Eigen::MatrixXd CXnl = GetXSelectorNl();
		Eigen::VectorXi wdep = GetWDep();
		Eigen::Index n = CXnl.rows();
		int m = wdep.sum();
		Eigen::Index l = n + m;

		Eigen::VectorXd z;
		Eigen::MatrixXd Sz;
		Eigen::MatrixXd Szx;
		Eigen::MatrixXd Szw;
		if (l == 0) {
			int ny = GetOutputSize(), nx = GetXInputSize(), nw = GetWInputSize();
			z = Eigen::VectorXd::Zero(ny);
			Sz = Eigen::MatrixXd::Zero(ny, ny);
			Szx = Eigen::MatrixXd::Zero(ny, nx);
			Szw = Eigen::MatrixXd::Zero(ny, nw);
		}
		else {
			Eigen::MatrixXd Snl = CXnl * x.variance * CXnl.transpose();
			Eigen::LLT<Eigen::MatrixXd> chol(Snl);
			Eigen::MatrixXd sqrtSnl = chol.matrixL();

			double alpha = 0.7;
			double beta = 2.;
			double kappa = 0.;
			double tau2 = alpha * alpha * (kappa + (double)l);
			double tau = sqrt(tau2);

			Eigen::MatrixXd dx = tau * x.variance * CXnl.transpose() * sqrtSnl.inverse().transpose();
			Eigen::MatrixXd dw = Eigen::MatrixXd::Zero(wdep.size(), m);
			unsigned int j = 0;
			for (Eigen::Index i = 0; i < wdep.size(); i++)
				if (wdep(i) == 1) {
					dw(i, j) = tau * sqrt(w.variance.diagonal()(i));
					j++;
				}

			Eigen::VectorXd z0 = EvalNl(x.vector, w.vector);
			std::vector<Eigen::VectorXd> zx = std::vector<Eigen::VectorXd>();
			*/
			/*
			std::cout << "dx" << dx << std::endl;
			for (unsigned int i = 0; i < dx.cols(); i++)
				std::cout << "dx" << i << ": " << dx.col(i) << std::endl;*/
		/*
			for (Eigen::Index i = 0; i < n; i++) {
				zx.push_back(EvalNl(x.vector + dx.col(i), w.vector));
				//std::cout << zx[i * 2] << std::endl;
				zx.push_back(EvalNl(x.vector - dx.col(i), w.vector));
				//std::cout << zx[i * 2 + 1] << std::endl;
			}
			std::vector<Eigen::VectorXd> zw = std::vector<Eigen::VectorXd>();
			for (Eigen::Index i = 0; i < m; i++) {
				zw.push_back(EvalNl(x.vector, w.vector + dw.col(i)));
				zw.push_back(EvalNl(x.vector, w.vector - dw.col(i)));
			}
			z = (tau2 - (double)l) / tau2 * z0;
			for (int i = 0; i < 2 * n; i++)
				z += zx[i] / 2. / tau2;
			for (int i = 0; i < 2 * m; i++)
				z += zw[i] / 2. / tau2;
				*/
			/*
			std::cout << "z" << z << std::endl;
			std::cout << "z0" << z0 << std::endl;*/
		/*
			Sz = (tau2 - l) / (tau2 + 1. + beta - alpha * alpha) * (z0 - z) * (z0 - z).transpose();
			Szx = Eigen::MatrixXd::Zero(z.size(), x.Length());
			Szw = Eigen::MatrixXd::Zero(z.size(), w.Length());
			for (unsigned int i = 0; i < n; i++) {
				Sz += (zx[i] - z) * (zx[i] - z).transpose() / 2. / tau2;
				Sz += (zx[i + n] - z) * (zx[i + n] - z).transpose() / 2. / tau2;
				Szx += (zx[2 * i] - zx[2 * i + 1])*dx.col(i).transpose() / 2. / tau2;
			}
			for (int i = 0; i < m; i++) {
				Sz += (zw[i] - z) * (zw[i] - z).transpose() / 2. / tau2;
				Sz += (zw[i + m] - z) * (zw[i + m] - z).transpose() / 2. / tau2;
				Szw += (zw[2 * i] - zw[2 * i + 1])*dw.col(i).transpose() / 2. / tau2;
			}
		}

		Eigen::MatrixXd A = GetA();
		Eigen::MatrixXd B = GetB();

		Eigen::VectorXd y = A * x.vector + B * w.vector + z;
		Eigen::MatrixXd temp = Szx * A.transpose() + Szw * B.transpose();
		Eigen::MatrixXd Sy = A * x.variance * A.transpose() + B * w.variance*B.transpose() +
			Sz + temp + temp.transpose();
		Syx = A * x.variance + Szx;
		Syw = B * w.variance + Szw;
		//std::cout << "y: " << y << "\n Syy: " << Sy << "\n Syx: " << Syx << "\n Syw: " << Syw << std::endl;

		return StatisticValue(y, Sy);
		
		
		// Get number of nonlin dependencies and selector matrices

		// Chol on nonlinparts

		// Get nonline result

		// Get coeff matrices

		

		*/
return StatisticValue();
	}

public:

	void saveMeasurement(unsigned int ID, Eigen::VectorXd value);

	SystemManager(BaseSystemData data, StatisticValue state_);

	~SystemManager();

	void AddSensor(SensorData sensorData, StatisticValue sensorState);
};



