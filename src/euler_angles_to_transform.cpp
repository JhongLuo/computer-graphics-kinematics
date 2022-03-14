#include "euler_angles_to_transform.h"
// Construct a rotation matrix (as a 4x4 transformation) given a set of Euler
// angles.
//
// Inputs:
//   xzx  3-vector of extrinsic Euler angles rotating about the x-, z-, and
//     x-axes.
// Returns 3d Eigen Affine transformation.
Eigen::Affine3d euler_angles_to_transform(
	const Eigen::Vector3d& xzx)
{
	/////////////////////////////////////////////////////////////////////////////
	double anglex1 = xzx[0] * (3.1415926 / 180.0);
	double anglez = xzx[1] * (3.1415926 / 180.0);
	double anglex2 = xzx[2] * (3.1415926 / 180.0);

	Eigen::Affine3d transformx1, transformz, transforx2;
	transformx1.matrix() << 1, 0, 0, 0,
		0, cos(anglex1), -sin(anglex1), 0,
		0, sin(anglex1), cos(anglex1), 0,
		0, 0, 0, 1;

	transformx1.matrix() << cos(anglez), -sin(anglez), 0, 0,
		sin(anglez), cos(anglez), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	transformx1.matrix() << 1, 0, 0, 0,
		0, cos(anglex2), -sin(anglex2), 0,
		0, sin(anglex2), cos(anglex2), 0,
		0, 0, 0, 1;


	return transforx2 * transformz * transformx1;
	/////////////////////////////////////////////////////////////////////////////
}
