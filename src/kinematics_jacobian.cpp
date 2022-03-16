#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
	const Skeleton& skeleton,
	const Eigen::VectorXi& b,
	Eigen::MatrixXd& J)
{
	/////////////////////////////////////////////////////////////////////////////
	 //https://github.com/alecjacobson/computer-graphics-kinematics#finite-differencing
	Eigen::VectorXd tips_pose = transformed_tips(skeleton, b);
	double h = 0.0000001;
	Skeleton skeleton_moved(skeleton);

	for (int j = 0; j < skeleton.size(); j++) {
		for (int theta = 0; theta < 3; theta++) {
			skeleton_moved[j].xzx[theta] += h;
			Eigen::VectorXd tips_moved_pose = transformed_tips(skeleton_moved, b);
			skeleton_moved[j].xzx[theta] -= h;

			for (int i = 0; i < b.size(); i++) {
				for (int axis = 0; axis < 3; axis++) {
					J(3 * i + axis, 3 * j + theta) = (tips_moved_pose[3 * i + axis] - tips_pose[3 * i + axis]) / h;
				}
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////
}
