#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
	const std::vector<std::pair<double, Eigen::Vector3d> >& keyframes,
	double t)
{
	/////////////////////////////////////////////////////////////////////////////
	if (keyframes.size() <= 0) {
		return Eigen::Vector3d(0.0, 0.0, 0.0);
	}
	else if (keyframes.size() == 1) {
		// stay still
		return keyframes[0].second;
	}
	else {
		//make it a period move
		t = fmod(t, keyframes[keyframes.size() - 1].first);

		unsigned int pos = 0;
		//the frame is in (pos - 1, pos] if 0 < pos < keyframes.size()
		while (pos < keyframes.size() && keyframes[pos].first < t) {
			pos++;
		}


		double t0, t1, t2, t3;
		Eigen::Vector3d P0, P1, P2, P3;
		if (pos > 1) {
			t0 = keyframes[pos - 2].first;
			P0 = keyframes[pos - 2].second;

			t1 = keyframes[pos - 1].first;
			P1 = keyframes[pos - 1].second;
		}
		else {
			t0 = keyframes[0].first;
			P0 = keyframes[0].second;

			t1 = keyframes[0].first;
			P1 = keyframes[0].second;
		}

		if (pos < keyframes.size() - 1) {
			t2 = keyframes[pos].first;
			P2 = keyframes[pos].second;

			t3 = keyframes[pos + 1].first;
			P3 = keyframes[pos + 1].second;
		}
		else {
			t2 = keyframes[keyframes.size() - 1].first;
			P2 = keyframes[keyframes.size() - 1].second;

			t3 = keyframes[keyframes.size() - 1].first;
			P3 = keyframes[keyframes.size() - 1].second;
		}

		if (t2 - t1 == 0) {
			return P2;
		}
		else {
			double _t = 0.25;

			Eigen::Vector3d a = P1;
			Eigen::Vector3d b = -_t * P0 + _t * P2;
			Eigen::Vector3d c = 2 * _t * P0 + (_t - 3) * P1 + (3 - 2 * _t) * P2 - _t * P3;
			Eigen::Vector3d d = -_t * P0 + (2 - _t) * P1 + (_t - 2) * P2 + _t * P3;

			double x = abs((t - t1) / (t2 - t1));
			return a + b * x + c * pow(x, 2) + d * pow(x, 3);
		}


	}



	/////////////////////////////////////////////////////////////////////////////
}
