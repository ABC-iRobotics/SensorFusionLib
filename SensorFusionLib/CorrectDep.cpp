#include "pch.h"
#include "CorrectDep.h"


Eigen::VectorXi CorrectLength(Eigen::Index newsize, const Eigen::RowVectorXi& in) {
	Eigen::Index oldsize = in.size();
	Eigen::VectorXi out = in;
	out.conservativeResize(newsize);
	for (Eigen::Index i = oldsize; i < newsize; i++) out(i) = 0;
	return out;
}
