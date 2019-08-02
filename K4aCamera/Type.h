#pragma once
#ifndef COMMON_TYPE_H
#define COMMON_TYPE_H

#include <cstdint>
#include <vector>

struct Intrinsic
{
	int32_t width;
	int32_t height;
	float	intrinsic[3 * 3];	// row major as in OpenCV
	float	distortion[12];

	float fx() const { return intrinsic[0]; }
	float fy() const { return intrinsic[4]; }
	float cx() const { return intrinsic[2]; }
	float cy() const { return intrinsic[5]; }

	float& fx() { return intrinsic[0]; }
	float& fy() { return intrinsic[4]; }
	float& cx() { return intrinsic[2]; }
	float& cy() { return intrinsic[5]; }
};

struct Extrinsic
{
	float	extrinsic[4 * 4];	// row major as in OpenCV

	Extrinsic Inverse() const {
		// simple case of matrix inversion
		const float* m = extrinsic;
		Extrinsic inv = {
			m[0], m[4], m[8], -(m[0] * m[3] + m[4] * m[7] + m[8] * m[11]),
			m[1], m[5], m[9], -(m[1] * m[3] + m[5] * m[7] + m[9] * m[11]),
			m[2], m[6], m[10], -(m[2] * m[3] + m[6] * m[7] + m[10] * m[11]),
			0, 0, 0, 1
		};
		return inv;
	}
};

struct Mesh
{
	size_t cntV;	// vertices count
	size_t cntF;	// indices count
	size_t szV;		// vertex buffer size
	size_t szF;		// index buffer size
	float* bufV;	// vertex buffer
	int* bufF;		// index buffer
};

struct Plane
{
	/* A plane is denoted as Ax + By + Cz + D = 0 */
	float A, B, C, D;
};

struct Point
{
	float x, y, z;
};

struct Pose3D
{
	int szPoints;
	std::vector<Point> points;
};

#endif /* COMMON_TYPE_H */
