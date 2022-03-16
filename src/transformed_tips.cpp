#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
	const Skeleton& skeleton,
	const Eigen::VectorXi& b)
{
	/////////////////////////////////////////////////////////////////////////////
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > T;
	forward_kinematics(skeleton, T);

	Eigen::VectorXd ans(b.size() * 3);
	for (int i = 0; i < b.size(); i++) {
		Eigen::Vector4d tmp_affine = T[b[i]] * skeleton[b[i]].rest_T * Eigen::Vector4d(skeleton[b[i]].length, 0.0, 0.0, 1.0);
		ans[i * 3] = tmp_affine.x() / tmp_affine.w();
		ans[i * 3 + 1] = tmp_affine.y() / tmp_affine.w();
		ans[i * 3 + 2] = tmp_affine.z() / tmp_affine.w();
	}
	return ans;
	/////////////////////////////////////////////////////////////////////////////
}
