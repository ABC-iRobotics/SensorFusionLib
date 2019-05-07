#include "pch.h"
#include "FunctionMerge.h"

Eigen::MatrixXd GetSelector(const Eigen::VectorXi& nonlinDep, bool nonlin) {
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
}

FunctionMerge::FunctionMerge(const Function2 & f0_) : f0(f0_), fi(std::vector<Function4>()) {}

void FunctionMerge::AddFunction(const Function4 & f) {
	fi.push_back(f);
}

Eigen::Index FunctionMerge::GetOutputSize() const {
	Eigen::Index out = f0.GetOutputSize();
	for (size_t i = 0; i < fi.size(); i++)
		out += fi[i].GetOutputSize();
	return out;
}

Eigen::Index FunctionMerge::GetXInputSize() const {
	Eigen::Index out = f0.GetX0Size();
	for (size_t i = 0; i < fi.size(); i++)
		out += fi[i].GetXiSize();
	return out;
}

Eigen::Index FunctionMerge::GetWInputSize() const {
	Eigen::Index out = f0.GetW0Size();
	for (size_t i = 0; i < fi.size(); i++)
		out += fi[i].GetWiSize();
	return out;
}

Eigen::VectorXd FunctionMerge::Eval(const Eigen::VectorXd & x, const Eigen::VectorXd & w) const {
	Eigen::VectorXd out(GetOutputSize());
	Eigen::Index nx = f0.GetX0Size(), nw = f0.GetW0Size(), ny = f0.GetOutputSize();
	Eigen::VectorXd temp, x0 = x.segment(0, nx), w0 = w.segment(0, nw);
	temp = f0.Eval(x0, w0);
	for (unsigned int i = 0; i < ny; i++)
		out(i) = temp(i);
	Eigen::Index dx, dw, dy;
	for (size_t i = 0; i < fi.size(); i++) {
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
	Eigen::Index nx = f0.GetX0Size(), nw = f0.GetW0Size(), ny = f0.GetOutputSize();
	Eigen::VectorXd temp, x0 = x.segment(0, nx), w0 = w.segment(0, nw);
	temp = f0.f_nl(x0, w0);
	for (Eigen::Index i = 0; i < ny; i++)
		out(i) = temp(i);
	Eigen::Index dx, dw, dy;
	for (size_t i = 0; i < fi.size(); i++) {
		dx = fi[i].GetXiSize();
		dy = fi[i].GetOutputSize();
		dw = fi[i].GetWiSize();
		temp = fi[i].f_nl(x0, w0, x.segment(nx, dx), w.segment(nw, dw));
		for (Eigen::Index j = 0; j < dy; j++)
			out(j + ny) = temp(j);
		nx += dx;
		ny += dy;
		nw += dw;
	}
	return out;
}

Eigen::MatrixXd FunctionMerge::GetA() const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(GetOutputSize(), GetXInputSize());
	Eigen::Index nx0 = f0.GetX0Size();
	Eigen::Index ny = f0.GetOutputSize();
	for (Eigen::Index i = 0; i < ny; i++)
		for (Eigen::Index j = 0; j < nx0; j++)
			out(i, j) = f0.A0(i, j);
	Eigen::Index nx = nx0;
	Eigen::Index dx, dy;
	for (size_t i = 0; i < fi.size(); i++) {
		dy = fi[i].GetOutputSize();
		for (Eigen::Index k = 0; k < dy; k++)
			for (Eigen::Index j = 0; j < nx0; j++)
				out(ny + k, j) = fi[i].A0(k, j);
		dx = fi[i].GetXiSize();
		for (Eigen::Index k = 0; k < dy; k++)
			for (Eigen::Index j = 0; j < dx; j++)
				out(ny + k, nx + j) = fi[i].Ai(k, j);
		ny += dy;
		nx += dx;
	}
	return out;
}

Eigen::MatrixXd FunctionMerge::GetB() const {
	Eigen::MatrixXd out = Eigen::MatrixXd::Zero(GetOutputSize(), GetWInputSize());
	Eigen::Index nw0 = f0.GetW0Size();
	Eigen::Index ny = f0.GetOutputSize();
	for (Eigen::Index i = 0; i < ny; i++)
		for (Eigen::Index j = 0; j < nw0; j++)
			out(i, j) = f0.B0(i, j);
	Eigen::Index nw = nw0;
	Eigen::Index dw, dy;
	for (size_t i = 0; i < fi.size(); i++) {
		dy = fi[i].GetOutputSize();
		for (Eigen::Index k = 0; k < dy; k++)
			for (Eigen::Index j = 0; j < nw0; j++)
				out(ny + k, j) = fi[i].B0(k, j);
		dw = fi[i].GetWiSize();
		for (Eigen::Index k = 0; k < dy; k++)
			for (Eigen::Index j = 0; j < dw; j++)
				out(ny + k, nw + j) = fi[i].Bi(k, j);
		ny += dy;
		nw += dw;
	}
	return out;
}

Eigen::VectorXi FunctionMerge::GetXDep() const {
	Eigen::VectorXi out(GetXInputSize());
	Eigen::Index nx0 = f0.GetX0Size();
	for (Eigen::Index i = 0; i < nx0; i++)
		out(i) = f0.x0Dep(i);
	Eigen::Index dx, nx = nx0;
	for (size_t i = 0; i < fi.size(); i++) {
		/*for (unsigned int j = 0; j < nx0; j++)
			if (fi[i].x0Dep(j) == 1)
				out(j) = 1;*/
		dx = fi[i].GetXiSize();
		for (Eigen::Index j = 0; j < dx; j++)
			out(nx + j) = fi[i].xiDep(j);
		nx += dx;
	}
	return out;
}

Eigen::VectorXi FunctionMerge::GetWDep() const {
	Eigen::VectorXi out(GetWInputSize());
	Eigen::Index nw0 = f0.GetW0Size();
	for (Eigen::Index i = 0; i < nw0; i++)
		out(i) = f0.w0Dep(i);
	Eigen::Index dw, nw = nw0;
	for (size_t i = 0; i < fi.size(); i++) {
		for (Eigen::Index j = 0; j < nw0; j++)
			if (fi[i].w0Dep(j) == 1)
				out(j) = 1;
		dw = fi[i].GetWiSize();
		for (Eigen::Index j = 0; j < dw; j++)
			out(nw + j) = fi[i].wiDep(j);
		nw += dw;
	}
	return out;
}

#include <iostream>

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
		/*
		std::cout << "dx" << dx << std::endl;
		for (unsigned int i = 0; i < dx.cols(); i++)
			std::cout << "dx" << i << ": " << dx.col(i) << std::endl;*/
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
		/*
		std::cout << "z" << z << std::endl;
		std::cout << "z0" << z0 << std::endl;*/
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
}
