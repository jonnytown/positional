#include "GeomUtil.h"
#include "Util.h"

namespace Positional
{
	void GeomUtil::nearestOnSegments(const Vec3 &a0, const Vec3 &a1, const Vec3 &b0, const Vec3 &b1, Vec3 &outA, Vec3 &outB)
	{
		Vec3 u = a1 - a0;
		Vec3 v = b1 - b0;
		Float ul2 = u.lengthSq();
		Float vl2 = v.lengthSq();

		if (ul2 < Math::Epsilon || vl2 < Math::Epsilon)
		{
			outA = a0;
			outB = b0;

			if (vl2 >= Math::Epsilon)
			{
				outB = nearestOnSegment(a0, b0, b1);
			}
			else if (ul2 >= Math::Epsilon)
			{
				outA = nearestOnSegment(b1, a0, a1);
			}
			return;
		}

		Vec3 r = b0 - a0;

		Float ru = r.dot(u);
		Float rv = r.dot(v);
		Float uu = u.dot(u);
		Float uv = u.dot(v);
		Float vv = v.dot(v);

		Float det = uu * vv - uv * uv;
		Float s, t;

		if (det < Math::Epsilon * uu * vv)
		{
			// segments are parallel
			s = Math::clamp(ru / uu, 0, 1);
			t = 0;
		}
		else
		{
			s = Math::clamp((ru * vv - rv * uv) / det, 0, 1);
			t = Math::clamp((ru * uv - rv * uu) / det, 0, 1);
		}

		Float S = Math::clamp((t * uv + ru) / uu, 0, 1);
		Float T = Math::clamp((s * uv - rv) / vv, 0, 1);

		outA = a0 + S * u;
		outB = b0 + T * v;
	}

	void GeomUtil::nearestOnRaySegment(const Vec3 &r0, const Vec3 &n, const Vec3 &c0, const Vec3 &c1, Vec3 &outA, Vec3 &outB)
	{
		Vec3 v = c1 - c0;
		Float vl2 = v.lengthSq();

		if (vl2 < Math::Epsilon)
		{
			outA = nearestOnRay(c0, r0, n);
			outB = c0;
			return;
		}

		Vec3 r = c0 - r0;

		Float ru = r.dot(n);
		Float rv = r.dot(v);
		Float uu = n.dot(n);
		Float uv = n.dot(v);
		Float vv = v.dot(v);

		Float det = uu * vv - uv * uv;
		Float s, t;

		if (det < 0.000001 * uu * vv)
		{
			// segments are parallel
			if (uv >= 0)
			{
				s = Math::max(ru / uu, 0);
				t = 0;
			}
			else
			{
				s = Math::max(-ru / uu, 0);
				t = 1;
			}
		}
		else
		{
			s = Math::max((ru * vv - rv * uv) / det, 0);
			t = Math::clamp((ru * uv - rv * uu) / det, 0, 1);
		}

		Float S = Math::max((t * uv + ru) / uu, 0);
		Float T = Math::clamp((s * uv - rv) / vv, 0, 1);

		outA = r0 + S * n;
		outB = c0 + T * v;
	}

	bool GeomUtil::raycastBox(const Vec3 &extents, const Vec3 &r0, const Vec3 &n, const Vec3 &ni, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{

		Float t1 = (-extents.x - r0.x) * ni.x;
		Float t2 = (extents.x - r0.x) * ni.x;
		Float t3 = (-extents.y - r0.y) * ni.y;
		Float t4 = (extents.y - r0.y) * ni.y;
		Float t5 = (-extents.z - r0.z) * ni.z;
		Float t6 = (extents.z - r0.z) * ni.z;

		Float tmin = Math::max(Math::max(Math::min(t1, t2), Math::min(t3, t4)), Math::min(t5, t6));
		Float tmax = Math::min(Math::min(Math::max(t1, t2), Math::max(t3, t4)), Math::max(t5, t6));

		if (tmin >= 0 && tmax >= tmin && (d <= 0 || tmin <= d))
		{
			outPoint = r0 + n * tmin;
			outNormal = Vec3(
							Math::approx(extents.x, Math::abs(outPoint.x), 0.00001) ? Math::sign(outPoint.x) : 0,
							Math::approx(extents.y, Math::abs(outPoint.y), 0.00001) ? Math::sign(outPoint.y) : 0,
							Math::approx(extents.z, Math::abs(outPoint.z), 0.00001) ? Math::sign(outPoint.z) : 0)
							.normalize();
			outDistance = tmin;

			return true;
		}

		return false;
	}

	bool GeomUtil::raycastSphere(const Vec3 &center, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{
		const Vec3 toCenter = center - r0;
		const Float r2 = radius * radius;
		const Float toCenterSq = toCenter.lengthSq();

		// origin starts inside sphere
		if (toCenterSq < r2)
		{
			return false;
		}

		const Vec3 projPoint = r0 + toCenter.project(n);
		const Float opSq = center.distanceSq(projPoint);
		if (opSq <= r2)
		{
			const Float t = Math::approx(opSq, 0) ? Math::sqrt(toCenterSq) - radius : r0.distance(projPoint) - Math::sqrt(r2 - opSq);
			if (d <= 0 || t <= d)
			{
				outPoint = r0 + n * t;
				outNormal = (outPoint - center).normalize();
				outDistance = t;
				return true;
			}
		}

		return false;
	}

	bool GeomUtil::raycastCylinder(const Vec3 &c0, const Vec3 &c1, const Float &length, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{
		Vec3 AB = c1 - c0;
		Vec3 AO = r0 - c0;

		// parallel
		if (Math::approx(Math::abs(AB.dot(n) / length), 1, 0.000001))
		{
			return false;
		}

		Vec3 AOxAB = AO.cross(AB);
		Vec3 VxAB = n.cross(AB);

		Float ab2 = length * length;
		Float A = VxAB.dot(VxAB);
		Float B = 2 * VxAB.dot(AOxAB);
		Float C = AOxAB.dot(AOxAB) - radius * radius * ab2;
		Float D = B * B - 4 * A * C;
		if (D < 0)
		{
			return false;
		}

		Float time = (-B - Math::sqrt(D)) / (2 * A);
		if (time < 0 || (d > 0 && time > d))
		{
			return false;
		}

		Vec3 intersection = r0 + n * time;
		Float projectionTime = AB.dot(intersection - c0) / ab2;

		if (projectionTime < 0 || projectionTime > 1)
		{
			return false;
		}

		Vec3 projection = c0 + AB * projectionTime;

		outPoint = intersection;
		outNormal = (intersection - projection).normalize();
		outDistance = time;

		return true;
	}

	bool GeomUtil::raycastCaps(const Vec3 &c0, const Vec3 &c1, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{
		Vec3 capPoint0, capNorm0, capPoint1, capNorm1;
		Float capDist0, capDist1;
		bool cap0 = GeomUtil::raycastSphere(c0, radius, r0, n, d, capPoint0, capNorm0, capDist0);
		bool cap1 = GeomUtil::raycastSphere(c1, radius, r0, n, d, capPoint1, capNorm1, capDist1);

		if (cap0 && cap1)
		{
			if (capPoint0.distanceSq(r0) < capPoint1.distanceSq(r0))
			{
				outPoint = capPoint0;
				outNormal = capNorm0;
				outDistance = capDist0;
			}
			else
			{
				outPoint = capPoint1;
				outNormal = capNorm1;
				outDistance = capDist1;
			}

			return true;
		}

		if (cap0)
		{
			outPoint = capPoint0;
			outNormal = capNorm0;
			outDistance = capDist0;
			return true;
		}

		if (cap1)
		{
			outPoint = capPoint1;
			outNormal = capNorm1;
			outDistance = capDist1;
			return true;
		}

		return false;
	}

	bool GeomUtil::raycastCapsule(const Vec3 &c0, const Vec3 &c1, const Float &length, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{
		// capsule is sphere
		if (c0 == c1)
		{
			return raycastSphere(c0, radius, r0, n, d, outPoint, outNormal, outDistance);
		}

		// ray starts inside capsule
		Vec3 r0p = nearestOnSegment(r0, c0, c1);
		if (r0p.distanceSq(r0) < radius * radius)
		{
			return false;
		}

		if (raycastCylinder(c0, c1, length, radius, r0, n, d, outPoint, outNormal, outDistance))
		{
			return true;
		}

		return raycastCaps(c0, c1, radius, r0, n, d, outPoint, outNormal, outDistance);
	}
}