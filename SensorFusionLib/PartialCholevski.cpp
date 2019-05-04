
#include "PartialCholevski.h"
#include <vector>
#include <iostream>

Eigen::MatrixXd PartialChol(Eigen::MatrixXd a, Eigen::VectorXi v) {
	const Eigen::Index size = a.rows();
	enum Status { NO_DECOMP, TO_BE_DECOMP, DECOMPOSED };
	std::vector<Status> status = std::vector<Status>();
	for (size_t i = 0; i < size; i++)
		status.push_back(v[i] == 1 ? TO_BE_DECOMP : NO_DECOMP);
	Eigen::MatrixXd out = Eigen::MatrixXd(size, v.sum());
	// Compute matrix L1 norm = max abs column sum.
	typedef Eigen::LLT<Eigen::MatrixXd>::RealScalar Real;
	using std::sqrt;
	int i_out_col = 0; // index for columns of out 
	for (Eigen::Index i_in_col = 0; i_in_col < size; ++i_in_col)
		if (status[i_in_col] == TO_BE_DECOMP) {
			for (Eigen::Index i_row = 0; i_row < size; i_row++)
				out(i_row, i_out_col) = status[i_row] != DECOMPOSED ? a(i_row, i_in_col) : 0;
			Eigen::Index remaining_size_row = size - i_out_col - 1; // remaining size
			Eigen::MatrixXd A10(1, i_out_col);
			Eigen::MatrixXd A20(remaining_size_row, i_out_col);
			Eigen::MatrixXd A21(remaining_size_row, 1);
			{
				int i_out = 0;
				for (Eigen::Index i_row = 0; i_row < size; i_row++)
					if (status[i_row] != DECOMPOSED && i_row != i_in_col) {
						A21(i_out, 0) = out(i_row, i_out_col);
						for (Eigen::Index i_col = 0; i_col < i_out_col; i_col++)
							A20(i_out, i_col) = out(i_row, i_col);
						i_out++;
					}
				for (Eigen::Index i_col = 0; i_col < i_out_col; i_col++)
					A10(0, i_col) = out(i_in_col, i_col);
			}
			Real x = Eigen::numext::real(out.coeff(i_in_col, i_out_col));
			if (i_out_col > 0) x -= A10.squaredNorm();
			if (x <= Real(0))
				throw std::runtime_error(std::string("The matrix is not positive definite or numerical error."));
			out.coeffRef(i_in_col, i_out_col) = x = sqrt(x);
			if (i_out_col > 0 && remaining_size_row > 0) A21.noalias() -= A20 * A10.adjoint();
			if (remaining_size_row > 0) A21 /= x;
			{
				int i_out = 0;
				for (Eigen::Index i_row = 0; i_row < size; i_row++)
					if (status[i_row] != DECOMPOSED && i_row != i_in_col) {
						out(i_row, i_out_col) = A21(i_out, 0);
						i_out++;
					}
			}
			status[i_in_col] = DECOMPOSED;
			i_out_col++;
		}
	return out;
}

void test() {
	Eigen::MatrixXd mi = Eigen::MatrixXd::Identity(3, 3) * 4;
	mi(0, 1) = 1; mi(1, 0) = 1;
	mi(2, 1) = 2; mi(1, 2) = 2;
	mi(2, 0) = -2; mi(0, 2) = -2;

	std::cout << mi << std::endl << std::endl;

	Eigen::VectorXi v = Eigen::VectorXi::Ones(3);
	v(2) = 0;
	v(1) = 1;
	v(0) = 1;
	Eigen::MatrixXd sqrtmi2 = PartialChol(mi, v);
	std::cout << sqrtmi2 << std::endl << std::endl;
	std::cout << sqrtmi2 * sqrtmi2.transpose() << std::endl << std::endl;
}

/*
inline Eigen::MatrixXd Sqrt(Eigen::MatrixXd a, Eigen::VectorXi v) {
	Eigen::MatrixXd out = a;
	const Eigen::Index size = a.rows();

	// Compute matrix L1 norm = max abs column sum.
	typedef Eigen::LLT<Eigen::MatrixXd>::RealScalar Real;

	using std::sqrt;

	bool error = false;

	for (Eigen::Index k = 0; k < size; k++)
	{
		//std::cout << k << std::endl;
		Eigen::Index rs = size - k - 1; // remaining size

		Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, 1> A21(out, k + 1, k, rs, 1);
		Eigen::Block<Eigen::MatrixXd, 1, Eigen::Dynamic> A10(out, k, 0, 1, k);
		Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> A20(out, k + 1, 0, rs, k);

		Real x = Eigen::numext::real(out.coeff(k, k));
		if (k > 0) x -= A10.squaredNorm();
		//std::cout << A10 << std::endl << "sq: " << A10.squaredNorm() << std::endl;
		if (x <= Real(0)) {
			error = true;
			break;
		}
		out.coeffRef(k, k) = x = sqrt(x);
		if (k > 0 && rs > 0) A21.noalias() -= A20 * A10.adjoint();
		if (rs > 0) A21 /= x;
		for (Eigen::Index k2 = 0; k2 < k; ++k2)
			out(k2, k) = 0;

		//std::cout << "Step " << k << std::endl << out << std::endl;
	}
	if (error)
		std::cout << "ERROR\n";

	return out;
}
*/