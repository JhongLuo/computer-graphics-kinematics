#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
	const Skeleton& skeleton,
	const Eigen::VectorXi& b,
	const Eigen::VectorXd& xb0,
	std::function<double(const Eigen::VectorXd&)>& f,
	std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& grad_f,
	std::function<void(Eigen::VectorXd&)>& proj_z)
{
	/////////////////////////////////////////////////////////////////////////////
	f = [&](const Eigen::VectorXd& A)->double
	{
		Eigen::VectorXd tips_pose = transformed_tips(copy_skeleton_at(skeleton, A), b);
		return (tips_pose - xb0).dot(tips_pose - xb0);
	};
	grad_f = [&](const Eigen::VectorXd& A)->Eigen::VectorXd
	{
		Skeleton skeleton_moved = copy_skeleton_at(skeleton, A);
		Eigen::MatrixXd Jacob = Eigen::MatrixXd::Zero(b.size() * 3, skeleton.size() * 3);
		kinematics_jacobian(skeleton_moved, b, Jacob);

		Eigen::VectorXd grad = Eigen::VectorXd::Zero(A.size());
		Eigen::VectorXd tips_pose = transformed_tips(skeleton_moved, b);
		for (int i = 0; i < b.size(); i++) {
			for (int j = 0; j < 3; j++) {
				//J * 2 * dE/dx
				grad += Jacob.row(3 * i + j).transpose() * 2 * (tips_pose(3 * i + j) - xb0(3 * i + j));
			}
		}
		return grad;
	};
	proj_z = [&](Eigen::VectorXd& A)
	{
		for (int i = 0; i < skeleton.size(); i++) {
			for (int j = 0; j < 3; j++) {
				if (A[3 * i + j] > skeleton[i].xzx_max[j]) {
					A[3 * i + j] = skeleton[i].xzx_max[j];
					continue;
				}
				if (A[3 * i + j] < skeleton[i].xzx_min[j]) {
					A[3 * i + j] = skeleton[i].xzx_min[j];
				}
			}
		}
	};
	/////////////////////////////////////////////////////////////////////////////
}
