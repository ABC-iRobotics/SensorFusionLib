#include "pch.h"
#include "FunctionMerge.h"

Eigen::MatrixXd GetSelector(const Eigen::VectorXi& nonlinDep, bool nonlin) {
	unsigned int outRows = nonlinDep.sum();
	if (!nonlin) outRows = nonlinDep.size() - outRows;
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(outRows, nonlinDep.size());
	unsigned int i = 0;
	for (unsigned int n = 0; n < nonlinDep.size(); n++)
		if ((nonlinDep[n] == 0) ^ nonlin) {
			out(i, n) = 1;
			i++;
		}
	return out;
}

FunctionMerge::FunctionMerge(const Function2 & f0_) : f0(f0_), fi(std::vector<Function4>()) {}

void FunctionMerge::AddFunction(const Function4 & f) {
	fi.push_back(f);
}

unsigned int FunctionMerge::GetOutputSize() const {
	unsigned int out = f0.GetOutputSize();
	for (unsigned int i = 0; i < fi.size(); i++)
		out += fi[i].GetOutputSize();
	return out;
}

unsigned int FunctionMerge::GetXInputSize() const {
	unsigned int out = f0.GetX0Size();
	for (unsigned int i = 0; i < fi.size(); i++)
		out += fi[i].GetXiSize();
	return out;
}

unsigned int FunctionMerge::GetWInputSize() const {
	unsigned int out = f0.GetW0Size();
	for (unsigned int i = 0; i < fi.size(); i++)
		out += fi[i].GetWiSize();
	return out;
}

Eigen::VectorXd FunctionMerge::Eval(const Eigen::VectorXd & x, const Eigen::VectorXd & w) const {
	Eigen::VectorXd out(GetOutputSize());
	unsigned int nx = f0.GetX0Size(), nw = f0.GetW0Size(), ny = f0.GetOutputSize();
	Eigen::VectorXd temp, x0 = x.segment(0, nx), w0 = w.segment(0, nw);
	temp = f0.Eval(x0, w0);
	for (unsigned int i = 0; i < ny; i++)
		out(i) = temp(i);
	unsigned int dx, dw, dy;
	for (unsigned int i = 0; i < fi.size(); i++) {
		dx = fi[i].GetXiSize();
		dy = fi[i].GetOutputSize();
		dw = fi[i].GetWiSize();
		temp = fi[i].Eval(x0, w0, x.segment(nx, dx), w.segment(nw, dw));
		for (unsigned int j = 0; j < dy; j++)
			out(j + ny) = temp(j);
		nx += dx;
		ny += dy;
		nw += dw;
	}
	return out;
}

Eigen::VectorXd FunctionMerge::EvalNl(const Eigen::VectorXd & x, const Eigen::VectorXd & w) const {
	Eigen::VectorXd out(GetOutputSize());
	unsigned int nx = f0.GetX0Size(), nw = f0.GetW0Size(), ny = f0.GetOutputSize();
	Eigen::VectorXd temp, x0 = x.segment(0, nx), w0 = w.segment(0, nw);
	temp = f0.f_nl(x0, w0);
	for (unsigned int i = 0; i < ny; i++)
		out(i) = temp(i);
	unsigned int dx, dw, dy;
	for (unsigned int i = 0; i < fi.size(); i++) {
		dx = fi[i].GetXiSize();
		dy = fi[i].GetOutputSize();
		dw = fi[i].GetWiSize();
		temp = fi[i].f_nl(x0, w0, x.segment(nx, dx), w.segment(nw, dw));
		for (unsigned int j = 0; j < dy; j++)
			out(j + ny) = temp(j);
		nx += dx;
		ny += dy;
		nw += dw;
	}
	return out;
}

Eigen::MatrixXd FunctionMerge::GetA() const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(GetOutputSize(), GetXInputSize());
	unsigned int nx0 = f0.GetX0Size();
	unsigned int ny = f0.GetOutputSize();
	for (unsigned int i = 0; i < ny; i++)
		for (unsigned int j = 0; j < nx0; j++)
			out(i, j) = f0.A0(i, j);
	unsigned int nx = nx0;
	unsigned int dx, dy;
	for (unsigned int i = 0; i < fi.size(); i++) {
		dy = fi[i].GetOutputSize();
		for (unsigned int k = 0; k < dy; k++)
			for (unsigned int j = 0; j < nx0; j++)
				out(ny + k, j) = fi[i].A0(k, j);
		dx = fi[i].GetXiSize();
		for (unsigned int k = 0; k < dy; k++)
			for (unsigned int j = 0; j < dx; j++)
				out(ny + k, nx + j) = fi[i].Ai(k, j);
		ny += dy;
		nx += dx;
	}
	return out;
}

Eigen::MatrixXd FunctionMerge::GetB() const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(GetOutputSize(), GetWInputSize());
	unsigned int nw0 = f0.GetW0Size();
	unsigned int ny = f0.GetOutputSize();
	for (unsigned int i = 0; i < ny; i++)
		for (unsigned int j = 0; j < nw0; j++)
			out(i, j) = f0.B0(i, j);
	unsigned int nw = nw0;
	unsigned int dw, dy;
	for (unsigned int i = 0; i < fi.size(); i++) {
		dy = fi[i].GetOutputSize();
		for (unsigned int k = 0; k < dy; k++)
			for (unsigned int j = 0; j < nw0; j++)
				out(ny + k, j) = fi[i].B0(k, j);
		dw = fi[i].GetWiSize();
		for (unsigned int k = 0; k < dy; k++)
			for (unsigned int j = 0; j < dw; j++)
				out(ny + k, nw + j) = fi[i].Bi(k, j);
		ny += dy;
		nw += dw;
	}
	return out;
}

Eigen::VectorXi FunctionMerge::GetXDep() const {
	Eigen::VectorXi out(GetXInputSize());
	unsigned int nx0 = f0.GetX0Size();
	for (unsigned int i = 0; i < nx0; i++)
		out(i) = f0.x0Dep(i);
	unsigned int dx, nx = nx0;
	for (unsigned int i = 0; i < fi.size(); i++) {
		for (unsigned int j = 0; j < nx0; j++)
			if (fi[i].x0Dep(j) == 1)
				out(j) = 1;
		dx = fi[i].GetXiSize();
		for (unsigned int j = 0; j < dx; j++)
			out(nx + j) = fi[i].xiDep(j);
		nx += dx;
	}
	return out;
}

Eigen::VectorXi FunctionMerge::GetWDep() const {
	Eigen::VectorXi out(GetWInputSize());
	unsigned int nw0 = f0.GetW0Size();
	for (unsigned int i = 0; i < nw0; i++)
		out(i) = f0.w0Dep(i);
	unsigned int dw, nw = nw0;
	for (unsigned int i = 0; i < fi.size(); i++) {
		for (unsigned int j = 0; j < nw0; j++)
			if (fi[i].w0Dep(j) == 1)
				out(j) = 1;
		dw = fi[i].GetWiSize();
		for (unsigned int j = 0; j < dw; j++)
			out(nw + j) = fi[i].wiDep(j);
		nw += dw;
	}
	return out;
}

Eigen::MatrixXd FunctionMerge::GetXSelectorNl() const {
	return GetSelector(GetXDep(), true);
}

Eigen::MatrixXd FunctionMerge::GetXSelectorL() const {
	return GetSelector(GetXDep(), false);
}

Eigen::MatrixXd FunctionMerge::GetWSelectorNl() const {
	return GetSelector(GetWDep(), true);
}

Eigen::MatrixXd FunctionMerge::GetWSelectorL() const {
	return GetSelector(GetWDep(), false);
}

StatisticValue FunctionMerge::Eval(const StatisticValue& x, const StatisticValue& w,
	Eigen::MatrixXd & Syx, Eigen::MatrixXd & Syw) const {

	Eigen::MatrixXd CXnl = GetXSelectorNl();
	Eigen::MatrixXd Snl = CXnl * x.variance * CXnl.transpose();
	Eigen::LLT<Eigen::MatrixXd> chol(Snl);
	Eigen::MatrixXd sqrtSnl = chol.matrixL();

	Eigen::VectorXi wdep = GetWDep();
	wdep(2) = 1;
	wdep(0) = 1;

	unsigned int n = Snl.cols();
	unsigned int m = wdep.sum();
	unsigned int l = n + m;
	double alpha = 0.7;
	double beta = 2.;
	double kappa = 0;
	double tau2 = alpha * alpha*(kappa + (double)l);
	double tau = sqrt(tau2);

	Eigen::MatrixXd dx = tau * x.variance * CXnl.transpose() * sqrtSnl.inverse().transpose();
	Eigen::MatrixXd dw = Eigen::MatrixXd::Zero(wdep.size(), m);
	unsigned int j = 0;
	for (unsigned int i = 0; i < wdep.size(); i++)
		if (wdep(i) == 1) {
			dw(i, j) = tau * sqrt(w.variance.diagonal()(i));
			j++;
		}

	Eigen::VectorXd z0 = EvalNl(x.vector, w.vector);
	std::vector<Eigen::VectorXd> zx = std::vector<Eigen::VectorXd>();
	for (unsigned int i = 0; i < n; i++) {
		zx.push_back(EvalNl(x.vector + dx.col(i), w.vector));
		zx.push_back(EvalNl(x.vector - dx.col(i), w.vector));
	}
	std::vector<Eigen::VectorXd> zw = std::vector<Eigen::VectorXd>();
	for (unsigned int i = 0; i < m; i++) {
		zx.push_back(EvalNl(x.vector, w.vector + dw.col(i)));
		zx.push_back(EvalNl(x.vector, w.vector - dw.col(i)));
	}

	Eigen::VectorXd z = (tau2 - (double)l) / tau2 * z0;
	for (unsigned int i = 0; i < 2 * n; i++)
		z += zx[i] / 2. / tau2;
	for (unsigned int i = 0; i < 2 * m; i++)
		z += zw[i] / 2. / tau2;

	Eigen::MatrixXd Sz = (tau2 - l) / (tau2 + 1. + beta - alpha * alpha) * (z0 - z) * (z0 - z).transpose();
	Eigen::MatrixXd Szx = Eigen::MatrixXd::Zero(z.size(), x.Length());
	Eigen::MatrixXd Szw = Eigen::MatrixXd::Zero(z.size(), w.Length());
	for (unsigned int i = 0; i < n; i++) {
		Sz += (zx[i] - z) * (zx[i] - z).transpose() / 2. / tau2;
		Sz += (zx[i + n] - z) * (zx[i + n] - z).transpose() / 2. / tau2;
		Szx += (zx[2 * i] - zx[2 * i + 1])*dx.col(i).transpose() / 2. / tau2;
	}
	for (unsigned int i = 0; i < m; i++) {
		Sz += (zw[i] - z) * (zw[i] - z).transpose() / 2. / tau2;
		Sz += (zw[i + n] - z) * (zw[i + n] - z).transpose() / 2. / tau2;
		Szw += (zw[2 * i] - zw[2 * i + 1])*dw.col(i).transpose() / 2. / tau2;
	}

	Eigen::MatrixXd A = GetA();
	Eigen::MatrixXd B = GetB();

	Eigen::VectorXd y = A * x.vector + B * w.vector + z;
	Eigen::MatrixXd temp = Szx * A.transpose() + Szw * B.transpose();
	Eigen::MatrixXd Sy = A * x.variance * A.transpose() + B * w.variance*B.transpose() +
		Sz + temp + temp.transpose();

	Syx = A * x.variance + Szx;
	Syw = B * w.variance + Szw;

	return StatisticValue(y, Sy);
}
