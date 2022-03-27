#pragma once
#ifndef __SPC_MAT33__
#define __SPC_MAT33__

#include <cassert>
#include <cmath>
#include "SPC_Constants.h"

namespace SP2C
{
	struct SPC_Mat33
	{
		double m[3][3];

		SPC_Mat33(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
		{
			m[0][0] = m00;
			m[0][1] = m01;
			m[0][2] = m02;

			m[1][0] = m10;
			m[1][1] = m11;
			m[1][2] = m12;

			m[2][0] = m20;
			m[2][1] = m21;
			m[2][2] = m22;
		}

		SPC_Mat33(double mat[3][3])
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					m[i][j] = mat[i][j];
		}

		SPC_Mat33()
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					m[i][j] = 0;
		}

		SPC_Mat33 operator+(SPC_Mat33 mat)
		{
			SPC_Mat33 ret;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					ret.m[i][j] = m[i][j] + mat.m[i][j];

			return ret;
		}

		SPC_Mat33 operator-(SPC_Mat33 mat)
		{
			SPC_Mat33 ret;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					ret.m[i][j] = m[i][j] - mat.m[i][j];

			return ret;
		}

		SPC_Mat33 operator*(double k)
		{
			SPC_Mat33 ret;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					ret.m[i][j] = m[i][j] * k;

			return ret;
		}

		SPC_Mat33 operator*(SPC_Mat33 mat)
		{
			SPC_Mat33 ret;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					for (int k = 0; k < 3; k++)
						ret.m[i][j] += m[i][k] * mat.m[k][j];

			return ret;
		}

		Vec2 operator*(Vec2 v)
		{
			return Vec2(m[0][0] * v.x + m[0][1] * v.y + m[0][2], m[1][0] * v.x + m[1][1] * v.y + m[1][2]);
		}

		SPC_Mat33& operator+=(SPC_Mat33 mat)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					m[i][j] += mat.m[i][j];

			return *this;
		}

		SPC_Mat33& operator-=(SPC_Mat33 mat)
		{;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					m[i][j] -= mat.m[i][j];

			return *this;
		}

		SPC_Mat33& operator*=(double k)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					m[i][j] *= k;

			return *this;
		}

		SPC_Mat33& operator=(SPC_Mat33 mat)
		{
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					m[i][j] = mat.m[i][j];

			return *this;
		}

		SPC_Mat33& operator*=(SPC_Mat33 mat)
		{
			SPC_Mat33 val;

			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					for (int k = 0; k < 3; k++)
						val.m[i][j] += m[i][k] * mat.m[k][j];

			*this = val;
			return *this;
		}

		SPC_Mat33 Transpose()
		{
			SPC_Mat33 ret;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					ret.m[i][j] = m[j][i];

			return ret;
		}

		double Determinant()
		{
			return m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
				   m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
				   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
		}

		SPC_Mat33 Inverse()
		{
			double det = Determinant();

			assert(det != 0);
			double invdet = 1 / det;

			SPC_Mat33 ret;

			ret.m[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
			ret.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
			ret.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
			ret.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
			ret.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
			ret.m[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
			ret.m[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
			ret.m[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
			ret.m[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;

			return ret;
		}

		void Translate(double x, double y)
		{
			m[0][2] += x;
			m[1][2] += y;
		}

		void Translate(Vec2 p)
		{
			m[0][2] += p.x;
			m[1][2] += p.y;
		}

		void Scale(double k)
		{
			m[0][0] *= k;
			m[1][1] *= k;
		}

		void Rotate(double deg)
		{
			double cos = std::cos(deg * Const::RAD);
			double sin = std::sin(deg * Const::RAD);

			*this *= SPC_Mat33(cos, -sin, 0, sin, cos, 0, 0, 0, 0);
		}
	};

	const SPC_Mat33 SPC_MAT_IDENTITY = SPC_Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1);
}
#endif