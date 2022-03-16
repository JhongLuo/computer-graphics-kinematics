#include "copy_skeleton_at.h"
Skeleton copy_skeleton_at(
	const Skeleton& skeleton,
	const Eigen::VectorXd& A)
{
	/////////////////////////////////////////////////////////////////////////////
	Skeleton skeleton_moved(skeleton);
	for (int i = 0; i < skeleton.size(); i++) {
		skeleton_moved[i].xzx.x() = A[i * 3];
		skeleton_moved[i].xzx.y() = A[i * 3 + 1];
		skeleton_moved[i].xzx.z() = A[i * 3 + 2];
	}
	return skeleton_moved;
	/////////////////////////////////////////////////////////////////////////////
}
