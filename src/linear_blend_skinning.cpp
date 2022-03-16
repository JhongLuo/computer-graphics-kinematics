#include "linear_blend_skinning.h"

void linear_blend_skinning(
	const Eigen::MatrixXd& V,
	const Skeleton& skeleton,
	const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >& T,
	const Eigen::MatrixXd& W,
	Eigen::MatrixXd& U)
{
	/////////////////////////////////////////////////////////////////////////////
	U.resize(V.rows(), 3);
	for (int i = 0; i < V.rows(); i++) {
		Eigen::Vector4d pose;
		pose.x() = 0;
		pose.y() = 0;
		pose.z() = 0;
		pose.w() = 0;
		for (int j = 0; j < skeleton.size(); j++) {
			if (skeleton[j].weight_index != -1) {
				pose += W(i, skeleton[j].weight_index) * (T[j] * Eigen::Vector4d(V(i, 0), V(i, 1), V(i, 2), 1));
			}
		}
		U(i, 0) = pose.x() / pose.w();
		U(i, 1) = pose.y() / pose.w();
		U(i, 2) = pose.z() / pose.w();
	}
	/////////////////////////////////////////////////////////////////////////////
}
