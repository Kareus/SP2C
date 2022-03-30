#pragma once
#ifndef __SPC_MAT33__
#define __SPC_MAT33__

#include <cassert>
#include <cmath>
#include "SPC_Constants.h"
#include "SPC_Vector.h"

namespace SP2C
{
	struct SPC_Mat33
	{
		double m[3][3];

		SPC_Mat33(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

		SPC_Mat33(double** mat);

		SPC_Mat33();

		SPC_Mat33 operator+(SPC_Mat33 mat);

		SPC_Mat33 operator-(SPC_Mat33 mat);

		SPC_Mat33 operator*(double k);

		SPC_Mat33 operator*(SPC_Mat33 mat);

		Vec2 operator*(Vec2 v);

		SPC_Mat33& operator+=(SPC_Mat33 mat);

		SPC_Mat33& operator-=(SPC_Mat33 mat);

		SPC_Mat33& operator*=(double k);

		SPC_Mat33& operator=(SPC_Mat33 mat);

		SPC_Mat33& operator*=(SPC_Mat33 mat);

		SPC_Mat33 Transpose();

		double Determinant();

		SPC_Mat33 Inverse();

		void Translate(double x, double y);

		void Translate(Vec2 p);

		void Scale(double k);

		void Rotate(double deg);
	};

	const SPC_Mat33 SPC_MAT_IDENTITY = SPC_Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1);
}
#endif