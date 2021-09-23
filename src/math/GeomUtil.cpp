#include "GeomUtil.h"
#include "Util.h"

namespace Positional
{
	Vec3 GeomUtil::barycentric(const Vec3 &point, const Vec3 &a, const Vec3 &b, const Vec3 &c)
	{
		const Vec3 u = b - a;
		const Vec3 v = c - a;
		const Vec3 n = u.cross(v);

		// nn is area of the triangle * 2
		const Float nn = n.lengthSq();

		// project on plane
		const Vec3 w = point-a;
		const Vec3 p = w - n * (w.dot(n) / nn);

		// calculate signed areas squared (area of the parellelagram formed by two vectors)
		const Vec3 uxp = u.cross(p);
		const Vec3 pxv = p.cross(v);

		const Float s = uxp.lengthSq() / nn;
		const Float t = pxv.lengthSq() / nn;

		const Float S = Math::sign(uxp.dot(n)) * Math::sqrt(s);
		const Float T = Math::sign(pxv.dot(n)) * Math::sqrt(t);

		return Vec3(1.0 - S - T, T, S);
	}

	void GeomUtil::nearestOnSegments(const Vec3 &a0, const Vec3 &a1, const Vec3 &b0, const Vec3 &b1, Vec3 &outA, Vec3 &outB)
	{
		const Vec3 u = a1 - a0;
		const Vec3 v = b1 - b0;
		const Float ul2 = u.lengthSq();
		const Float vl2 = v.lengthSq();

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

		const Vec3 r = b0 - a0;

		const Float ru = r.dot(u);
		const Float rv = r.dot(v);
		const Float uu = u.dot(u);
		const Float uv = u.dot(v);
		const Float vv = v.dot(v);

		const Float det = uu * vv - uv * uv;
		Float s, t;

		if (det < Math::Epsilon * uu * vv)
		{
			const Vec3 w = b1 - a0;
			const Float wu = w.dot(u);
			// segments are parallel
			s = 0.5 * (Math::clamp(ru / uu, 0, 1) + Math::clamp(wu / uu, 0, 1));

			const Vec3 x = (a0 + u * s) - b0;
			const Float xv = x.dot(v);

			t = Math::clamp(xv / vv, 0, 1);
		}
		else
		{
			s = Math::clamp((ru * vv - rv * uv) / det, 0, 1);
			t = Math::clamp((ru * uv - rv * uu) / det, 0, 1);
		}

		const Float S = Math::clamp((t * uv + ru) / uu, 0, 1);
		const Float T = Math::clamp((s * uv - rv) / vv, 0, 1);

		outA = a0 + S * u;
		outB = b0 + T * v;
	}

	void GeomUtil::nearestOnRaySegment(const Vec3 &r0, const Vec3 &n, const Vec3 &c0, const Vec3 &c1, Vec3 &outA, Vec3 &outB)
	{
		const Vec3 v = c1 - c0;
		const Float vl2 = v.lengthSq();

		if (vl2 < Math::Epsilon)
		{
			outA = nearestOnRay(c0, r0, n);
			outB = c0;
			return;
		}

		const Vec3 r = c0 - r0;

		const Float ru = r.dot(n);
		const Float rv = r.dot(v);
		const Float uu = n.dot(n);
		const Float uv = n.dot(v);
		const Float vv = v.dot(v);

		const Float det = uu * vv - uv * uv;
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

		const Float S = Math::max((t * uv + ru) / uu, 0);
		const Float T = Math::clamp((s * uv - rv) / vv, 0, 1);

		outA = r0 + S * n;
		outB = c0 + T * v;
	}

	bool GeomUtil::raycastBox(const Vec3 &extents, const Vec3 &r0, const Vec3 &n, const Vec3 &ni, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{

		const Float t1 = (-extents.x - r0.x) * ni.x;
		const Float t2 = (extents.x - r0.x) * ni.x;
		const Float t3 = (-extents.y - r0.y) * ni.y;
		const Float t4 = (extents.y - r0.y) * ni.y;
		const Float t5 = (-extents.z - r0.z) * ni.z;
		const Float t6 = (extents.z - r0.z) * ni.z;

		const Float tmin = Math::max(Math::max(Math::min(t1, t2), Math::min(t3, t4)), Math::min(t5, t6));
		const Float tmax = Math::min(Math::min(Math::max(t1, t2), Math::max(t3, t4)), Math::max(t5, t6));

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
		if (toCenterSq >= r2)
		{
			// will project in front of origin
			const Float uv = toCenter.dot(n);
			if (uv > 0)
			{
				const Vec3 projPoint = r0 + n*uv;
				const Float opSq = center.distanceSq(projPoint);
				if (opSq <= r2)
				{
					const Float t = Math::approx(opSq, 0) ? Math::sqrt(toCenterSq) - radius : uv - Math::sqrt(r2 - opSq);
					if (d <= 0 || t <= d)
					{
						outPoint = r0 + n * t;
						outNormal = (outPoint - center).normalize();
						outDistance = t;
						return true;
					}
				}
			}
		}

		return false;
	}

	bool GeomUtil::raycastCylinder(const Vec3 &c0, const Vec3 &c1, const Float &length, const Float &radius, const Vec3 &r0, const Vec3 &n, const Float &d, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{
		const Vec3 AB = c1 - c0;
		const Vec3 AO = r0 - c0;

		// parallel
		if (Math::approx(Math::abs(AB.dot(n) / length), 1, 0.000001))
		{
			return false;
		}

		const Vec3 AOxAB = AO.cross(AB);
		const Vec3 VxAB = n.cross(AB);

		const Float ab2 = length * length;
		const Float A = VxAB.dot(VxAB);
		const Float B = 2 * VxAB.dot(AOxAB);
		const Float C = AOxAB.dot(AOxAB) - radius * radius * ab2;
		const Float D = B * B - 4 * A * C;
		if (D < 0)
		{
			return false;
		}

		const Float time = (-B - Math::sqrt(D)) / (2 * A);
		if (time < 0 || (d > 0 && time > d))
		{
			return false;
		}

		const Vec3 intersection = r0 + n * time;
		const Float projectionTime = AB.dot(intersection - c0) / ab2;

		if (projectionTime < 0 || projectionTime > 1)
		{
			return false;
		}

		const Vec3 projection = c0 + AB * projectionTime;

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

	bool GeomUtil::raycastTriangle(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &r0, const Vec3 &rn, const Float &maxDist, Vec3 &outPoint, Vec3 &outNormal, Float &outDistance)
	{
		const Vec3 u = b - a;
		const Vec3 v = c - a;
		const Vec3 n = u.cross(v);

		const Float nrn = n.dot(rn);
		if (nrn < 0)
		{
			// ray is parallel or in same direction as normal
			return false;
		}

		const Float t = (n.dot(r0) + n.dot(a)) / nrn;
		if (t >= 0 && (maxDist <= 0 || t < maxDist))
		{
			const Vec3 p = r0 + rn * t;
			const Vec3 uxp = u.cross(p);
			const Vec3 pxv = p.cross(v);

			// calculate signed areas squared (area of the parellelagram formed by two vectors)
			const Float area = n.lengthSq();
			const Float areaU = uxp.lengthSq() * Math::sign(uxp.dot(n));
			const Float areaV = pxv.lengthSq() * Math::sign(pxv.dot(n));

			// squared barycentric coords
			const Float S = areaU / area;
			const Float T = areaV / area;

			if (S >= 0 && T >= 0 && Math::sqrt(S) + Math::sqrt(T) <= 1)
			{
				outPoint = p;
				outNormal = n;
				outDistance = t;
				return true;
			}
		}

		return false;
	}
}