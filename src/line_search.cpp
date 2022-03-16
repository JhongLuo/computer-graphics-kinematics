#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
	double step_size = max_step;
	double E = f(z);
	Eigen::VectorXd new_z = z - step_size * dz;
	proj_z(new_z);
	unsigned int iter = 0;
	while (true) {
		step_size *= 0.5;
		new_z = z - step_size * dz;
		proj_z(new_z);
		if (f(new_z) < E) {
			return step_size;
		}
		//0.5^20 = 1e-6
		if (++iter > 20) {
			return step_size;
		}
	}
  /////////////////////////////////////////////////////////////////////////////
}
