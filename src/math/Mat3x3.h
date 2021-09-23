/* 
 * Row-major 3x3 matrix class
 */
#include "Primitives.h"
#include "Util.h"
#include "Vec3.h"
#include "Quat.h"
#include <assert.h>

#ifndef MAT3X3_H
#define MAT3X3_H

namespace Positional
{
	class Mat3x3
	{
	private:
		Float m[9];
	public:

		Mat3x3() {
			m[0] = 1.0;
			m[1] = 0.0;
			m[2] = 0.0;
			m[3] = 0.0;
			m[4] = 1.0;
			m[5] = 0.0;
			m[6] = 0.0;
			m[7] = 0.0;
			m[8] = 1.0;
		}

		Mat3x3(const Float &v)
		{
			m[0] = v;
			m[1] = v;
			m[2] = v;
			m[3] = v;
			m[4] = v;
			m[5] = v;
			m[6] = v;
			m[7] = v;
			m[8] = v;
		}

		Mat3x3(
			const Float &m11, const Float &m12, const Float &m13,
			const Float &m21, const Float &m22, const Float &m23,
			const Float &m31, const Float &m32, const Float &m33)
		{
			m[0] = m11;
			m[1] = m12;
			m[2] = m13;
			m[3] = m21;
			m[4] = m22;
			m[5] = m23;
			m[6] = m31;
			m[7] = m32;
			m[8] = m33;
		}

		Mat3x3(const Vec3 &c1, const Vec3 &c2, const Vec3 &c3)
		{
			m[0] = c1.x;
			m[1] = c2.x;
			m[2] = c3.x;
			m[3] = c1.y;
			m[4] = c2.y;
			m[5] = c3.y;
			m[6] = c1.z;
			m[7] = c2.z;
			m[8] = c3.z;
		}

		Mat3x3(const Quat &quat)
		{
			/*
			const Float xx = quat.x * quat.x;
			const Float yy = quat.y * quat.y;
			const Float zz = quat.z * quat.z;

			const Float xy = quat.x * quat.y;
			const Float wz = quat.z * quat.w;
			const Float xz = quat.z * quat.x;
			const Float wy = quat.y * quat.w;
			const Float yz = quat.y * quat.z;
			const Float wx = quat.x * quat.w;

			return Mat3x3(
				1.0 - 2.0 * (yy + zz),
				2.0 * (xy + wz),
				2.0 * (xz - wy),
				2.0 * (xy - wz),
				1.0 - 2.0 * (zz + xx),
				2.0 * (yz + wx),
				2.0 * (xz + wy),
				2.0 * (yz - wx),
				1.0 - 2.0 * (yy + xx)
			);
			*/

			const Float x = quat.x;
			const Float y = quat.y;
			const Float z = quat.z;
			const Float w = quat.w;

			const Float x2 = x + x;
			const Float y2 = y + y;
			const Float z2 = z + z;

			const Float xx = x2 * x;
			const Float yy = y2 * y;
			const Float zz = z2 * z;

			const Float xy = x2 * y;
			const Float xz = x2 * z;
			const Float xw = x2 * w;

			const Float yz = y2 * z;
			const Float yw = y2 * w;
			const Float zw = z2 * w;

			m[0] = 1.0 - yy - zz; m[1] = xy - zw;       m[2] = xz + yw;
			m[3] = xy + zw;       m[4] = 1.0 - xx - zz; m[5] = yz - xw;
			m[6] = xz - yw;       m[7] = yz + xw;       m[8] = 1.0 - xx - yy;
		}

		Mat3x3(const Float &angle, const Vec3 &axis)
		{
			const Float x = axis.x, y = axis.y, z = axis.z;
			const Float sa = Math::sin(angle), ca = Math::cos(angle);
			const Float xx = x * x, yy = y * y, zz = z * z;
			const Float xy = x * y, xz = x * z, yz = y * z;

			m[0] = xx + ca * (1.0 - xx); m[1] = xy - ca * xy + sa * z; m[2] = xz - ca * xz - sa * y;
			m[3] = xy - ca * xy - sa * z; m[4] = yy + ca * (1.0 - yy); m[5] = yz - ca * yz + sa * x;
			m[6] = xz - ca * xz + sa * y; m[7] = yz - ca * yz - sa * x; m[8] = zz + ca * (1.0 - zz);
		}

		Float m11() const { return m[0]; }
		Float m12() const { return m[1]; }
		Float m13() const { return m[2]; }
		Float m21() const { return m[3]; }
		Float m22() const { return m[4]; }
		Float m23() const { return m[5]; }
		Float m31() const { return m[6]; }
		Float m32() const { return m[7]; }
		Float m33() const { return m[8]; }

		Mat3x3 &set(
			const Float &m11, const Float &m12, const Float &m13,
			const Float &m21, const Float &m22, const Float &m23,
			const Float &m31, const Float &m32, const Float &m33)
		{
			m[0] = m11;
			m[1] = m12;
			m[2] = m13;
			m[3] = m21;
			m[4] = m22;
			m[5] = m23;
			m[6] = m31;
			m[7] = m32;
			m[8] = m33;
			return *this;
		}

		Float get(const UInt8 &r, const UInt8 &c) const
		{
			assert(r < 3);
			assert(c < 3);
			return m[r * 3 + c];
		}

		Mat3x3 & set(const Float &v, const UInt8 &r, const UInt8 &c)
		{
			assert(r < 3);
			assert(c < 3);
			m[r * 3 + c] = v;
			return *this;
		}

		Mat3x3 & operator=(const Mat3x3 &rhs)
		{
			std::memcpy(m, rhs.m, sizeof(Float) * 9);
			return *this;
		}

		Float &operator[](const UInt32 &i)
		{
			assert(i < 9);
			return m[i];
		}

		bool operator==(const Mat3x3 &rhs) const
		{
			return Math::approx(m[0], rhs.m[0]) &&
				Math::approx(m[1], rhs.m[1]) &&
				Math::approx(m[2], rhs.m[2]) &&
				Math::approx(m[3], rhs.m[3]) &&
				Math::approx(m[4], rhs.m[4]) &&
				Math::approx(m[5], rhs.m[5]) &&
				Math::approx(m[6], rhs.m[6]) &&
				Math::approx(m[7], rhs.m[7]) &&
				Math::approx(m[8], rhs.m[8]);
		}

		bool operator!=(const Mat3x3 &rhs) const
		{
			return ! Math::approx(m[0], rhs.m[0])
				|| ! Math::approx(m[1], rhs.m[1])
				|| ! Math::approx(m[2], rhs.m[2])
				|| ! Math::approx(m[3], rhs.m[3])
				|| ! Math::approx(m[4], rhs.m[4])
				|| ! Math::approx(m[5], rhs.m[5])
				|| ! Math::approx(m[6], rhs.m[6])
				|| ! Math::approx(m[7], rhs.m[7])
				|| ! Math::approx(m[8], rhs.m[8]);
		}

		bool isIdentity() const
		{
			return m[0] == 1.0 && m[4] == 1.0 && m[8] == 1.0
				&& m[1] == 0.0 && m[2]
				&& m[3] == 0.0 && m[5]
				&& m[6] == 0.0 && m[7];
		}

		Vec3 getColumn(const UInt8 &c) const
		{
			assert(c < 3);
			return Vec3(m[0 + c], m[3 + c], m[6 + c]);
		}

		Mat3x3 & setColumn(const UInt8 &c, const Vec3 &v)
		{
			assert(c < 3);
			m[0 + c] = v.x;
			m[3 + c] = v.y;
			m[6 + c] = v.z;
			return *this;
		}

		Float determinant() const
		{
			Float a = m[0], b = m[1], c = m[2];
			Float d = m[3], e = m[4], f = m[5];
			Float g = m[6], h = m[7], i = m[8];

			Float ei_fh = e * i - f * h;
			Float di_fg = d * i - f * g;
			Float dh_eg = d * h - e * g;

			return a * ei_fh - b * di_fg + c * dh_eg;
		}

		Mat3x3 inverse() const
		{
			Float a = m[0], b = m[1], c = m[2];
			Float d = m[3], e = m[4], f = m[5];
			Float g = m[6], h = m[7], i = m[8];

			Float ei_fh = e * i - f * h;
			Float di_fg = d * i - f * g;
			Float dh_eg = d * h - e * g;

			Float det = a * ei_fh - b * di_fg + c * dh_eg;

			if (Math::abs(det < Math::Epsilon))
			{
				return Mat3x3(Math::NaN, Math::NaN, Math::NaN, Math::NaN, Math::NaN, Math::NaN, Math::NaN, Math::NaN, Math::NaN);
			}

			Float invDet = 1.0 / det;

			return Mat3x3(
				invDet * (e * i - f * h),
				invDet * -(b * i - h * c),
				invDet * (b * f - e * c),

				invDet * -(d * i - f * g),
				invDet * (a * i - g * c),
				invDet * -(a * f - d * c),

				invDet * (d * h - e * g),
				invDet * -(a * h - g * b),
				invDet * (a * e - b * d));
		}

		Mat3x3 & invert()
		{
			Float a = m[0], b = m[1], c = m[2];
			Float d = m[3], e = m[4], f = m[5];
			Float g = m[6], h = m[7], i = m[8];

			Float ei_fh = e * i - f * h;
			Float di_fg = d * i - f * g;
			Float dh_eg = d * h - e * g;

			Float det = a * ei_fh - b * di_fg + c * dh_eg;

			if (Math::abs(det < Math::Epsilon))
			{
				m[0] = m[1] = m[2] = m[3] = m[4] = m[5] = m[6] = m[7] = m[8] = Math::NaN;
			}

			Float invDet = 1.0 / det;

			m[0] = invDet * (e * i - f * h);
			m[1] = invDet * -(b * i - h * c);
			m[2] = invDet * (b * f - e * c);

			m[3] = invDet * -(d * i - f * g);
			m[4] = invDet * (a * i - g * c);
			m[5] = invDet * -(a * f - d * c);

			m[6] = invDet * (d * h - e * g);
			m[7] = invDet * -(a * h - g * b);
			m[8] = invDet * (a * e - b * d);

			return *this;
		}

		Mat3x3 transposed() const
		{
			return Mat3x3(
				m[0], m[3], m[6],
				m[1], m[4], m[7],
				m[2], m[5], m[8]);
		}

		Mat3x3 & transpose()
		{
			Float buf;

			buf = m[1];
			m[1] = m[3];
			m[3] = buf;

			buf = m[2];
			m[2] = m[6];
			m[6] = buf;

			buf = m[5];
			m[5] = m[7];
			m[7] = buf;

			return * this;
		}

		Mat3x3 operator+(const Mat3x3 &rhs) const
		{
			return Mat3x3(
				m[0] + rhs.m[0],
				m[1] + rhs.m[1],
				m[2] + rhs.m[2],

				m[3] + rhs.m[3],
				m[4] + rhs.m[4],
				m[5] + rhs.m[5],

				m[6] + rhs.m[6],
				m[7] + rhs.m[7],
				m[8] + rhs.m[8]);
		}

		Mat3x3 operator-(const Mat3x3 &rhs) const
		{
			return Mat3x3(
				m[0] - rhs.m[0],
				m[1] - rhs.m[1],
				m[2] - rhs.m[2],

				m[3] - rhs.m[3],
				m[4] - rhs.m[4],
				m[5] - rhs.m[5],

				m[6] - rhs.m[6],
				m[7] - rhs.m[7],
				m[8] - rhs.m[8]);
		}

		Mat3x3 operator*(const Mat3x3 &rhs) const
		{
			/*
			|m11=0, m12=1, m13=2|
			|m21=3, m22=4, m23=5|
			|m31=6, m32=7, m33=8|
			*/
			return Mat3x3(
				m[0] * rhs.m[0] + m[1] * rhs.m[3] + m[2] * rhs.m[6],
				m[0] * rhs.m[1] + m[1] * rhs.m[4] + m[2] * rhs.m[7],
				m[0] * rhs.m[2] + m[1] * rhs.m[5] + m[2] * rhs.m[8],

				m[3] * rhs.m[0] + m[4] * rhs.m[3] + m[5] * rhs.m[6],
				m[3] * rhs.m[1] + m[4] * rhs.m[4] + m[5] * rhs.m[7],
				m[3] * rhs.m[2] + m[4] * rhs.m[5] + m[5] * rhs.m[8],

				m[6] * rhs.m[0] + m[7] * rhs.m[3] + m[8] * rhs.m[6],
				m[6] * rhs.m[1] + m[7] * rhs.m[4] + m[8] * rhs.m[7],
				m[6] * rhs.m[2] + m[7] * rhs.m[5] + m[8] * rhs.m[8]);
		}

		Vec3 operator*(const Vec3 &rhs) const
		{
			return Vec3(
				rhs.x * m[0] + rhs.y * m[3] + rhs.z * m[6],
				rhs.x * m[1] + rhs.y * m[4] + rhs.z * m[7],
				rhs.x * m[2] + rhs.y * m[5] + rhs.z * m[8]);
		}

		/*
		 * 1x3 mutilplied by 3x3
		 */
		friend Vec3 operator*(const Vec3 &lhs, const Mat3x3 &rhs)
		{
			return Vec3(
				lhs.x * rhs.m[0] + lhs.y * rhs.m[1] + lhs.z * rhs.m[2],
				lhs.x * rhs.m[3] + lhs.y * rhs.m[4] + lhs.z * rhs.m[5],
				lhs.x * rhs.m[6] + lhs.y * rhs.m[7] + lhs.z * rhs.m[8]);
		}

		Mat3x3 operator*(const Float &rhs) const
		{
			return Mat3x3(
				m[0] * rhs, m[1] * rhs, m[2] * rhs,
				m[3] * rhs, m[4] * rhs, m[5] * rhs,
				m[6] * rhs, m[7] * rhs, m[8] * rhs);
		}

		friend Mat3x3 operator*(const Float &lhs, const Mat3x3 &rhs)
		{
			return Mat3x3(
				lhs * rhs.m[0], lhs * rhs.m[1], lhs * rhs.m[2],
				lhs * rhs.m[3], lhs * rhs.m[4], lhs * rhs.m[5],
				lhs * rhs.m[6], lhs * rhs.m[7], lhs * rhs.m[8]);
		}

		Mat3x3 operator/(const Float &rhs) const
		{
			return Mat3x3(
				m[0] / rhs, m[1] / rhs, m[2] / rhs,
				m[3] / rhs, m[4] / rhs, m[5] / rhs,
				m[6] / rhs, m[7] / rhs, m[8] / rhs);
		}

		friend Mat3x3 operator/(const Float &lhs, const Mat3x3 &rhs)
		{
			return Mat3x3(
				lhs / rhs.m[0], lhs / rhs.m[1], lhs / rhs.m[2],
				lhs / rhs.m[3], lhs / rhs.m[4], lhs / rhs.m[5],
				lhs / rhs.m[6], lhs / rhs.m[7], lhs / rhs.m[8]);
		}

		static const Mat3x3 identity;
	};
}

#endif // MAT3X3_H