/* 
 * Row-major 4x4 matrix class.
 */
#include "Primitives.h"
#include "Util.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Quat.h"
#include <assert.h>

#ifndef MAT4X4_H
#define MAT4X4_H

namespace Positional
{
	class Mat4x4
	{
	private:
		Float m[16];
	public:

		Mat4x4() {
			m[0] = 1.0;
			m[1] = 0.0;
			m[2] = 0.0;
			m[3] = 0.0;

			m[4] = 0.0;
			m[5] = 1.0;
			m[6] = 0.0;
			m[7] = 0.0;

			m[8] = 0.0;
			m[9] = 0.0;
			m[10] = 1.0;
			m[11] = 0.0;

			m[12] = 0.0;
			m[13] = 0.0;
			m[14] = 0.0;
			m[15] = 1.0;
		}

		Mat4x4(
			const Float m11, const Float m12, const Float m13, const Float m14,
			const Float m21, const Float m22, const Float m23, const Float m24,
			const Float m31, const Float m32, const Float m33, const Float m34,
			const Float m41, const Float m42, const Float m43, const Float m44)
		{
			m[0] = m11;
			m[1] = m12;
			m[2] = m13;
			m[3] = m14;

			m[4] = m21;
			m[5] = m22;
			m[6] = m23;
			m[7] = m24;

			m[7] = m31;
			m[8] = m32;
			m[9] = m33;
			m[10] = m34;

			m[11] = m41;
			m[12] = m42;
			m[13] = m43;
			m[14] = m44;
		}

		Mat4x4(const Float _m[16])
		{
			m[0] = _m[0];
			m[1] = _m[1];
			m[2] = _m[2];
			m[3] = _m[3];
			m[4] = _m[4];
			m[5] = _m[5];
			m[6] = _m[6];
			m[7] = _m[7];
			m[8] = _m[8];
			m[9] = _m[9];
			m[10] = _m[10];
			m[11] = _m[11];
			m[12] = _m[12];
			m[13] = _m[13];
			m[14] = _m[14];
			m[15] = _m[15];
		}

		Mat4x4(const Vec4 &c1, const Vec4 &c2, const Vec4 &c3, const Vec4 &c4)
		{
			m[0] = c1.x;
			m[1] = c2.x;
			m[2] = c3.x;
			m[3] = c4.x;

			m[4] = c1.y;
			m[5] = c2.y;
			m[6] = c3.y;
			m[7] = c4.y;

			m[8] = c1.z;
			m[9] = c2.z;
			m[10] = c3.z;
			m[11] = c4.z;

			m[12] = c1.w;
			m[13] = c2.w;
			m[14] = c3.w;
			m[15] = c4.w;
		}

		Float m11() const { return m[0]; }
		Float m12() const { return m[1]; }
		Float m13() const { return m[2]; }
		Float m14() const { return m[3]; }
		Float m21() const { return m[4]; }
		Float m22() const { return m[5]; }
		Float m23() const { return m[6]; }
		Float m24() const { return m[7]; }
		Float m31() const { return m[8]; }
		Float m32() const { return m[9]; }
		Float m33() const { return m[10]; }
		Float m34() const { return m[11]; }
		Float m41() const { return m[12]; }
		Float m42() const { return m[13]; }
		Float m43() const { return m[14]; }
		Float m44() const { return m[15]; }

		Float get(const UInt8 &r, const UInt8 &c) const
		{
			assert(r < 4);
			assert(c < 4);
			return m[r * 4 + c];
		}

		Mat4x4 & set(const Float &v, const UInt8 &r, const UInt8 &c)
		{
			assert(r < 4);
			assert(c < 4);
			m[r * 4 + c] = v;
			return *this;
		}

		Mat4x4 & operator=(const Mat4x4 &rhs)
		{
			m[0] = rhs.m[0];
			m[1] = rhs.m[1];
			m[2] = rhs.m[2];
			m[3] = rhs.m[3];
			m[4] = rhs.m[4];
			m[5] = rhs.m[5];
			m[6] = rhs.m[6];
			m[7] = rhs.m[7];
			m[8] = rhs.m[8];
			m[9] = rhs.m[9];
			m[10] = rhs.m[10];
			m[11] = rhs.m[11];
			m[12] = rhs.m[12];
			m[13] = rhs.m[13];
			m[14] = rhs.m[14];
			m[15] = rhs.m[15];
			return *this;
		}

		bool operator==(const Mat4x4 &rhs) const
		{
			return Math::approx(m[0], rhs.m[0])
				&& Math::approx(m[1], rhs.m[1])
				&& Math::approx(m[2], rhs.m[2])
				&& Math::approx(m[3], rhs.m[3])
				&& Math::approx(m[4], rhs.m[4])
				&& Math::approx(m[5], rhs.m[5])
				&& Math::approx(m[6], rhs.m[6])
				&& Math::approx(m[7], rhs.m[7])
				&& Math::approx(m[8], rhs.m[8])
				&& Math::approx(m[9], rhs.m[9])
				&& Math::approx(m[10], rhs.m[10])
				&& Math::approx(m[11], rhs.m[11])
				&& Math::approx(m[12], rhs.m[12])
				&& Math::approx(m[13], rhs.m[13])
				&& Math::approx(m[14], rhs.m[14])
				&& Math::approx(m[15], rhs.m[15]);
		}

		bool operator!=(const Mat4x4 &rhs) const
		{
			return !Math::approx(m[0], rhs.m[0])
				|| !Math::approx(m[1], rhs.m[1])
				|| !Math::approx(m[2], rhs.m[2])
				|| !Math::approx(m[3], rhs.m[3])
				|| !Math::approx(m[4], rhs.m[4])
				|| !Math::approx(m[5], rhs.m[5])
				|| !Math::approx(m[6], rhs.m[6])
				|| !Math::approx(m[7], rhs.m[7])
				|| !Math::approx(m[8], rhs.m[8])
				|| !Math::approx(m[9], rhs.m[9])
				|| !Math::approx(m[10], rhs.m[10])
				|| !Math::approx(m[11], rhs.m[11])
				|| !Math::approx(m[12], rhs.m[12])
				|| !Math::approx(m[13], rhs.m[13])
				|| !Math::approx(m[14], rhs.m[14])
				|| !Math::approx(m[15], rhs.m[15]);
		}

		bool isIdentity() const
		{
			return m[0] == 1.0 && m[5] == 1.0 && m[10] == 1.0 && m[15] == 1.0
				&& m[1] == 0.0 && m[2] == 0.0 && m[3] == 0.0
				&& m[4] == 0.0 && m[6] == 0.0 && m[7] == 0.0
				&& m[8] == 0.0 && m[9] == 0.0 && m[11] == 0.0
				&& m[12] == 0.0 && m[13] == 0.0 && m[14] == 0.0;
		}

		Vec4 getColumn(const UInt8 &c) const
		{
			assert(c < 4);
			return Vec4(m[0 + c], m[4 + c], m[8 + c], m[12 + c]);
		}

		Mat4x4 & setColumn(const UInt8 &c, const Vec4 &v)
		{
			assert(c < 4);
			m[0 + c] = v.x;
			m[4 + c] = v.y;
			m[8 + c] = v.z;
			m[12 + c] = v.w;
			return *this;
		}

		Float determinant() const
		{
			Float a = m[0], b = m[1], c = m[2], d = m[3];
			Float e = m[4], f = m[5], g = m[6], h = m[7];
			Float i = m[8], j = m[9], k = m[10], l = m[11];
			Float _m = m[12], n = m[13], o = m[14], p = m[15];

			Float kp_lo = k * p - l * o;
			Float jp_ln = j * p - l * n;
			Float jo_kn = j * o - k * n;
			Float ip_lm = i * p - l * _m;
			Float io_km = i * o - k * _m;
			Float in_jm = i * n - j * _m;

			return  a * (f * kp_lo - g * jp_ln + h * jo_kn) -
					b * (e * kp_lo - g * ip_lm + h * io_km) +
					c * (e * jp_ln - f * ip_lm + h * in_jm) -
					d * (e * jo_kn - f * io_km + g * in_jm);
		}

		Mat4x4 inverse() const
		{
			Float a = m[0], b = m[1], c = m[2], d = m[3];
			Float e = m[4], f = m[5], g = m[6], h = m[7];
			Float i = m[8], j = m[9], k = m[10], l = m[11];
			Float _m = m[12], n = m[13], o = m[14], p = m[15];

			Float kp_lo = k * p - l * o;
			Float jp_ln = j * p - l * n;
			Float jo_kn = j * o - k * n;
			Float ip_lm = i * p - l * _m;
			Float io_km = i * o - k * _m;
			Float in_jm = i * n - j * _m;

			Float a11 = +(f * kp_lo - g * jp_ln + h * jo_kn);
			Float a12 = -(e * kp_lo - g * ip_lm + h * io_km);
			Float a13 = +(e * jp_ln - f * ip_lm + h * in_jm);
			Float a14 = -(e * jo_kn - f * io_km + g * in_jm);

			Float det = a * a11 + b * a12 + c * a13 + d * a14;

			if (Math::abs(det < Math::Epsilon))
			{
				return Mat4x4(
					Math::NaN, Math::NaN, Math::NaN, Math::NaN,
					Math::NaN, Math::NaN, Math::NaN, Math::NaN,
					Math::NaN, Math::NaN, Math::NaN, Math::NaN,
					Math::NaN, Math::NaN, Math::NaN, Math::NaN);
			}

			Float invDet = 1.0 / det;

			Float gp_ho = g * p - h * o;
			Float fp_hn = f * p - h * n;
			Float fo_gn = f * o - g * n;
			Float ep_hm = e * p - h * _m;
			Float eo_gm = e * o - g * _m;
			Float en_fm = e * n - f * _m;

			Float gl_hk = g * l - h * k;
			Float fl_hj = f * l - h * j;
			Float fk_gj = f * k - g * j;
			Float el_hi = e * l - h * i;
			Float ek_gi = e * k - g * i;
			Float ej_fi = e * j - f * i;

			return Mat4x4(
				a11 * invDet,
				-(b * kp_lo - c * jp_ln + d * jo_kn) * invDet,
				+(b * gp_ho - c * fp_hn + d * fo_gn) * invDet,
				-(b * gl_hk - c * fl_hj + d * fk_gj) * invDet,

				a12 * invDet,
				+(a * kp_lo - c * ip_lm + d * io_km) * invDet,
				-(a * gp_ho - c * ep_hm + d * eo_gm) * invDet,
				+(a * gl_hk - c * el_hi + d * ek_gi) * invDet,

				a13 * invDet,
				-(a * jp_ln - b * ip_lm + d * in_jm) * invDet,
				+(a * fp_hn - b * ep_hm + d * en_fm) * invDet,
				-(a * fl_hj - b * el_hi + d * ej_fi) * invDet,
				
				a14 * invDet,
				+(a * jo_kn - b * io_km + c * in_jm) * invDet,
				-(a * fo_gn - b * eo_gm + c * en_fm) * invDet,
				+(a * fk_gj - b * ek_gi + c * ej_fi) * invDet);
		}

		Mat4x4 & invert()
		{
			Float a = m[0], b = m[1], c = m[2], d = m[3];
			Float e = m[4], f = m[5], g = m[6], h = m[7];
			Float i = m[8], j = m[9], k = m[10], l = m[11];
			Float _m = m[12], n = m[13], o = m[14], p = m[15];

			Float kp_lo = k * p - l * o;
			Float jp_ln = j * p - l * n;
			Float jo_kn = j * o - k * n;
			Float ip_lm = i * p - l * _m;
			Float io_km = i * o - k * _m;
			Float in_jm = i * n - j * _m;

			Float a11 = +(f * kp_lo - g * jp_ln + h * jo_kn);
			Float a12 = -(e * kp_lo - g * ip_lm + h * io_km);
			Float a13 = +(e * jp_ln - f * ip_lm + h * in_jm);
			Float a14 = -(e * jo_kn - f * io_km + g * in_jm);

			Float det = a * a11 + b * a12 + c * a13 + d * a14;

			if (Math::abs(det < Math::Epsilon))
			{
				m[0] = m[1] = m[2] = m[3] =
				m[4] = m[5] = m[6] = m[7] =
				m[8] = m[9] = m[10] = m[11] =
				m[12] = m[13] = m[14] = m[15] = Math::NaN;
				return *this;
			}

			Float invDet = 1.0 / det;

			Float gp_ho = g * p - h * o;
			Float fp_hn = f * p - h * n;
			Float fo_gn = f * o - g * n;
			Float ep_hm = e * p - h * _m;
			Float eo_gm = e * o - g * _m;
			Float en_fm = e * n - f * _m;

			Float gl_hk = g * l - h * k;
			Float fl_hj = f * l - h * j;
			Float fk_gj = f * k - g * j;
			Float el_hi = e * l - h * i;
			Float ek_gi = e * k - g * i;
			Float ej_fi = e * j - f * i;

			m[0] = a11 * invDet;
			m[1] = -(b * kp_lo - c * jp_ln + d * jo_kn) * invDet;
			m[2] = +(b * gp_ho - c * fp_hn + d * fo_gn) * invDet;
			m[3] = -(b * gl_hk - c * fl_hj + d * fk_gj) * invDet;

			m[4] = a12 * invDet,
			m[5] = +(a * kp_lo - c * ip_lm + d * io_km) * invDet;
			m[6] = -(a * gp_ho - c * ep_hm + d * eo_gm) * invDet;
			m[7] = +(a * gl_hk - c * el_hi + d * ek_gi) * invDet;

			m[8] = a13 * invDet,
			m[9] = -(a * jp_ln - b * ip_lm + d * in_jm) * invDet;
			m[10] = +(a * fp_hn - b * ep_hm + d * en_fm) * invDet;
			m[11] = -(a * fl_hj - b * el_hi + d * ej_fi) * invDet;

			m[12] = a14 * invDet,
			m[13] = +(a * jo_kn - b * io_km + c * in_jm) * invDet;
			m[14] = -(a * fo_gn - b * eo_gm + c * en_fm) * invDet;
			m[15] = +(a * fk_gj - b * ek_gi + c * ej_fi) * invDet;

			return *this;
		}

		Mat4x4 transposed() const
		{
			return Mat4x4(
				m[0], m[4], m[8], m[12],
				m[1], m[5], m[9], m[13],
				m[2], m[6], m[10], m[14],
				m[3], m[7], m[11], m[15]);
		}

		Mat4x4 & transpose()
		{
			Float buf;

			buf = m[1];
			m[1] = m[4];
			m[4] = buf;

			buf = m[2];
			m[2] = m[8];
			m[8] = buf;

			buf = m[3];
			m[3] = m[12];
			m[12] = buf;

			buf = m[6];
			m[6] = m[9];
			m[9] = buf;

			buf = m[7];
			m[7] = m[13];
			m[13] = buf;

			buf = m[11];
			m[11] = m[14];
			m[14] = buf;

			return * this;
		}

		Mat4x4 operator*(const Mat4x4 &rhs)
		{
			return Mat4x4(
				m[0] * rhs.m[0] + m[1] * rhs.m[4] + m[2] * rhs.m[8] + m[3] * rhs.m[12],
				m[0] * rhs.m[1] + m[1] * rhs.m[5] + m[2] * rhs.m[9] + m[3] * rhs.m[13],
				m[0] * rhs.m[2] + m[1] * rhs.m[6] + m[2] * rhs.m[10] + m[3] * rhs.m[14],
				m[0] * rhs.m[3] + m[1] * rhs.m[7] + m[2] * rhs.m[11] + m[3] * rhs.m[15],

				m[4] * rhs.m[0] + m[5] * rhs.m[4] + m[6] * rhs.m[8] + m[7] * rhs.m[12],
				m[4] * rhs.m[1] + m[5] * rhs.m[5] + m[6] * rhs.m[9] + m[7] * rhs.m[13],
				m[4] * rhs.m[2] + m[5] * rhs.m[6] + m[6] * rhs.m[10] + m[7] * rhs.m[14],
				m[4] * rhs.m[3] + m[5] * rhs.m[7] + m[6] * rhs.m[11] + m[7] * rhs.m[15],

				m[8] * rhs.m[0] + m[9] * rhs.m[4] + m[10] * rhs.m[8] + m[11] * rhs.m[12],
				m[8] * rhs.m[1] + m[9] * rhs.m[5] + m[10] * rhs.m[9] + m[11] * rhs.m[13],
				m[8] * rhs.m[2] + m[9] * rhs.m[6] + m[10] * rhs.m[10] + m[11] * rhs.m[14],
				m[8] * rhs.m[3] + m[9] * rhs.m[7] + m[10] * rhs.m[11] + m[11] * rhs.m[15],

				m[12] * rhs.m[0] + m[13] * rhs.m[4] + m[14] * rhs.m[8] + m[15] * rhs.m[12],
				m[12] * rhs.m[1] + m[13] * rhs.m[5] + m[14] * rhs.m[9] + m[15] * rhs.m[13],
				m[12] * rhs.m[2] + m[13] * rhs.m[6] + m[14] * rhs.m[10] + m[15] * rhs.m[14],
				m[12] * rhs.m[3] + m[13] * rhs.m[7] + m[14] * rhs.m[11] + m[15] * rhs.m[15]);
		}

		Vec3 operator*(const Vec3 &rhs)
		{
			return Vec3(
				rhs.x * m[0] + rhs.y * m[4] + rhs.z * m[8] + m[12],
				rhs.x * m[1] + rhs.y * m[5] + rhs.z * m[9] + m[13],
				rhs.x * m[2] + rhs.y * m[6] + rhs.z * m[10] + m[14]);
		}

		Vec4 operator*(const Vec4 &rhs)
		{
			return Vec4(
				rhs.x * m[0] + rhs.y * m[4] + rhs.z * m[8] + rhs.w * m[12],
				rhs.x * m[1] + rhs.y * m[5] + rhs.z * m[9] + rhs.w * m[13],
				rhs.x * m[2] + rhs.y * m[6] + rhs.z * m[10] + rhs.w * m[14],
				rhs.x * m[3] + rhs.y * m[7] + rhs.z + m[11] + rhs.w * m[15]);
		}

		friend Vec4 operator*(const Vec4 &lhs, const Mat4x4 &rhs)
		{
			return Vec4(
				lhs.x * rhs.m[0] + lhs.y * rhs.m[1] + lhs.z * rhs.m[2] + lhs.w * rhs.m[3],
				lhs.x * rhs.m[4] + lhs.y * rhs.m[5] + lhs.z * rhs.m[6] + lhs.w * rhs.m[7],
				lhs.x * rhs.m[8] + lhs.y * rhs.m[9] + lhs.z * rhs.m[10] + lhs.w * rhs.m[11],
				lhs.x * rhs.m[12] + lhs.y * rhs.m[13] + lhs.z + rhs.m[14] + lhs.w * rhs.m[15]);
		}

		static inline Mat4x4 fromTRS(const Vec3 &t, const Quat &r, const Vec3 &s = Vec3::one)
		{
			Float xx = r.x * r.x;
			Float yy = r.y * r.y;
			Float zz = r.z * r.z;

			Float xy = r.x * r.y;
			Float wz = r.z * r.w;
			Float xz = r.z * r.x;
			Float wy = r.y * r.w;
			Float yz = r.y * r.z;
			Float wx = r.x * r.w;

			return Mat4x4(
				(1.0 - 2.0 * (yy + zz)) * s.x,
				(2.0 * (xy + wz)) * s.y,
				(2.0 * (xz - wy)) * s.z,
				0.0,

				(2.0 * (xy - wz)) * s.x,
				(1.0 - 2.0 * (zz + xx)) * s.y,
				(2.0 * (yz + wx)) * s.z,
				0.0,

				(2.0 * (xz + wy)) * s.x,
				((yz - wx)) * s.y,
				(1.0 - 2.0 * (yy + xx)) * s.z,
				0.0,
				
				t.x, t.y, t.z, 1
				);
		}

		static const Mat4x4 identity;
	};
}

#endif // MAT4X4_H