#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function


void recur_set_T(
	const unsigned int idx,
	const Skeleton& skeleton,
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& T,
	std::vector<bool>& flags)
{
	const int parent = skeleton[idx].parent_index;
	if (parent == -1) {
		T[idx] = Eigen::Affine3d::Identity();
		flags[idx] = true;
		return;
	}
	else {
		if (!flags[parent]) {
			recur_set_T(parent, skeleton, T, flags);
		}
		T[idx] = Eigen::Affine3d(T[parent] * skeleton[idx].rest_T * euler_angles_to_transform(skeleton[idx].xzx) * skeleton[idx].rest_T.inverse());
		flags[idx] = true;
		return;
	}
}

void forward_kinematics(
	const Skeleton& skeleton,
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& T)
{
	/////////////////////////////////////////////////////////////////////////////
	T.resize(skeleton.size());
	std::vector<bool> flags(skeleton.size(), false);
	for (int i = 0; i < skeleton.size(); i++) {
		if (!flags[i]) {
			recur_set_T(i, skeleton, T, flags);
		}
	}
	/////////////////////////////////////////////////////////////////////////////
}
