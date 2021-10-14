#include "Penetration.h"
#include "simulation/Body.h"
#include "collision/collider/Collider.h"
#include "collision/collider/BoxCollider.h"
#include "collision/collider/SphereCollider.h"
#include "collision/collider/CapsuleCollider.h"

namespace Positional::Collision
{
	const Float k_gjk_epsilonSq = Math::Epsilon;
	const Float k_epa_epsilonSq = 0.000000001;

	inline void makeContact(
		const Collider &a,
		const Collider &b,
		const Vec3 &center,
		const Vec3 &radius,
		const Vec3 &normal,
		const Float &depth,
		ContactPoint &outContact)
	{
		outContact.normal = normal;
		outContact.depth = depth;
		outContact.pointA = Body::pointToLocal(a.body(), center + normal * (radius - depth));
		outContact.pointB = Body::pointToLocal(b.body(), center + normal * radius);
	}

	inline void makeContact(
		const Collider &a,
		const Collider &b,
		const Vec3 &center,
		const Vec3 &radius,
		const Vec3 &normal,
		const Float &depth,
		bool swap,
		ContactPoint &outContact)
	{
		makeContact(a, b, center, radius, normal, depth, outContact);
		if (swap)
		{
			outContact.normal = -outContact.normal;
			std::swap(outContact.pointA, outContact.pointB);
		}
	}

	bool Penetration::sphereSphere(const Collider &a, const Collider &b, ContactPoint &outContact)
	{
		const Vec3 bCenter = b.pointToWorld(Vec3::zero);
		const Vec3 toA = a.pointToWorld(Vec3::zero) - bCenter;
		const Float rs = (a.shape.radius + b.shape.radius);
		const Float lenSq = toA.lengthSq();
		if (lenSq < rs * rs)
		{
			const Float len = Math::sqrt(lenSq);
			makeContact(a, b, bCenter, b.shape.radius, toA * (1 / len), rs - len, outContact);
			return true;
		}
		return false;
	}

	bool Penetration::capsuleCapsule(const Collider &a, const Collider &b, ContactPoint &outContact)
	{
		const Float al_2 = a.shape.length / 2;
		const Vec3 a0 = a.pointToWorld(Vec3(0, -al_2, 0));
		const Vec3 a1 = a.pointToWorld(Vec3(0, al_2, 0));

		const Float bl_2 = b.shape.length / 2;
		const Vec3 b0 = b.pointToWorld(Vec3(0, -bl_2, 0));
		const Vec3 b1 = b.pointToWorld(Vec3(0, bl_2, 0));

		Vec3 nearA, nearB;
		GeomUtil::nearestOnSegments(a0, a1, b0, b1, nearA, nearB);
		const Vec3 toA = nearA - nearB;
		const Float rs = (a.shape.radius + b.shape.radius);
		const Float lenSq = toA.lengthSq();

		if (lenSq < rs * rs)
		{
			const Float len = Math::sqrt(lenSq);
			makeContact(a, b, nearB, b.shape.radius, toA * (1 / len), rs - len, outContact);
			return true;
		}

		return false;
	}

	bool Penetration::sphereCapsule(const Collider &sphere, const Collider &capsule, const bool &swapped, ContactPoint &outContact)
	{
		const Vec3 c = sphere.pointToWorld(Vec3::zero);

		const Float bl_2 = capsule.shape.length / 2;
		const Vec3 b0 = capsule.pointToWorld(Vec3(0, -bl_2, 0));
		const Vec3 b1 = capsule.pointToWorld(Vec3(0, bl_2, 0));

		const Vec3 b = GeomUtil::nearestOnSegment(c, b0, b1);
		const Vec3 toC = c - b;
		const Float rs = (sphere.shape.radius + capsule.shape.radius);
		const Float lenSq = toC.lengthSq();

		if (lenSq < rs * rs)
		{
			const Float len = Math::sqrt(lenSq);
			makeContact(sphere, capsule, b, capsule.shape.radius, toC * (1 / len), rs - len, swapped, outContact);
			return true;
		}

		return false;
	}

	inline void testBoxNorm(const Float &test, const Vec3 &passNorm, Float &ioMin, Vec3 &outNorm)
	{
		if (test < ioMin)
		{
			ioMin = test;
			outNorm = passNorm;
		}
	}

	bool Penetration::boxSphere(const Collider &box, const Collider &sphere, const bool &swapped, ContactPoint &outContact)
	{
		const Vec3 centerInWorld = sphere.pointToWorld(Vec3::zero);
		const Vec3 centerInBoxSpace = box.pointToLocal(centerInWorld);

		const Bounds bounds(Vec3::zero, box.shape.extents);

		if (bounds.contains(centerInBoxSpace))
		{
			Vec3 norm;
			const Vec3 &e = bounds.extents();
			Float min = FLOAT_MAX;
			testBoxNorm(Math::abs(e.x - centerInBoxSpace.x), Vec3::neg_x, min, outContact.normal);
			testBoxNorm(Math::abs(-e.x - centerInBoxSpace.x), Vec3::pos_x, min, outContact.normal);
			testBoxNorm(Math::abs(e.y - centerInBoxSpace.y), Vec3::neg_y, min, outContact.normal);
			testBoxNorm(Math::abs(-e.y - centerInBoxSpace.y), Vec3::pos_y, min, outContact.normal);
			testBoxNorm(Math::abs(e.z - centerInBoxSpace.y), Vec3::neg_z, min, outContact.normal);
			testBoxNorm(Math::abs(-e.z - centerInBoxSpace.y), Vec3::pos_z, min, outContact.normal);

			makeContact(
				box,
				sphere,
				centerInWorld,
				sphere.shape.radius,
				box.vectorToWorld(outContact.normal),
				sphere.shape.radius + min,
				swapped,
				outContact);
			return true;
		}

		const Vec3 closest = bounds.nearest(centerInBoxSpace);
		const Vec3 toClosest = closest - centerInBoxSpace;
		const Float lenSq = toClosest.lengthSq();
		if (lenSq < sphere.shape.radius * sphere.shape.radius)
		{
			const Float len = Math::sqrt(lenSq);
			makeContact(
				box,
				sphere,
				centerInWorld,
				sphere.shape.radius,
				box.vectorToWorld(toClosest * (1 / len)),
				sphere.shape.radius - len,
				swapped,
				outContact);
			return true;
		}
		return false;
	}

	inline void csoSupport(const Collider &a, const Collider &b, const Vec3 &axis, Vec3 &outSupport, Vec3 &outSupportA, Vec3 &outSupportB)
	{
		const Vec3 axisA = a.vectorToLocal(axis);
		const Vec3 axisB = b.vectorToLocal(-axis);

		outSupportA = a.localSupport(axisA);
		outSupportB = b.localSupport(axisB);

		outSupport = a.pointToWorld(outSupportA) - b.pointToWorld(outSupportB);
	}

	inline UInt8 leastSignificantComponent(const Vec3 &v)
	{
		if (v.x <= v.y && v.x <= v.z)
		{
			return 0;
		}

		if (v.y < v.x && v.y <= v.z)
		{
			return 1;
		}

		return 2;
	}

	bool Penetration::gjk_epa(const Collider &a, const Collider &b, ContactPoint &outContact)
	{
		Simplex simplex;
		const bool gjkPass = gjk(a, b, simplex);
		if (gjkPass)
		{
			Polytope polytope(simplex.vertices, simplex.verticesA, simplex.verticesB);
			epa(a, b, simplex.count, polytope, outContact);
			return true;
		}
		return false;
	}

	bool Penetration::gjk(const Collider& a, const Collider& b, Simplex& outSimplex)
	{
		// find initial support vertext using arbitrary first axis
		Vec3 support, supportA, supportB;
		csoSupport(a, b, Vec3::pos_x, support, supportA, supportB);

		const UInt8 MAX_ITERS = 16;
		for (UInt8 i = 0; i < MAX_ITERS; ++i)
		{
			outSimplex.add(support, supportA, supportB);

			UInt8 nearDim, nearIndex;
			const Vec3 nearest = outSimplex.nearest(nearDim, nearIndex);

			// nearest is origin: we have a collision
			if (nearest.lengthSq() < k_gjk_epsilonSq)
			{
				return true;
			}

			outSimplex.reduce(nearDim, nearIndex);

			// negated nearest is vector to origin
			const Vec3 search = -nearest.normalized();
			csoSupport(a, b, search, support, supportA, supportB);

			const Float supportDot = search.dot(support);
			const Float nearestDot = search.dot(nearest);

			if (supportDot <= nearestDot)
			{
				return false;
			}
		}
		return false;
	}

	void Penetration::epa(const Collider &a, const Collider &b, const UInt8 &count, Polytope &outPolytope, ContactPoint &outContact)
	{
		// expand cso to tetrahedron if it is not alread
		switch (count)
		{
		case 1:
			{
				static const Vec3 searches[6] = {
					Vec3::neg_x, Vec3::pos_x, Vec3::neg_y, Vec3::pos_y, Vec3::neg_z, Vec3::pos_z};

				for (UInt8 i = 0; i < 6; ++i)
				{
					csoSupport(a, b, searches[i], outPolytope.vertices[1], outPolytope.verticesA[1], outPolytope.verticesB[1]);
					if (outPolytope.vertices[0].distanceSq(outPolytope.vertices[1]) > k_epa_epsilonSq)
					{
						break;
					}
				}
			}
			[[fallthrough]]; //end point to segment
		case 2:
			{
				static const Vec3 axes[3] = {Vec3::pos_x, Vec3::pos_y, Vec3::pos_z};
				static const Float pi_3 = Math::Pi / 3.0;

				const Vec3 v = outPolytope.vertices[1] - outPolytope.vertices[0];

				const UInt8 leastSigAxis = leastSignificantComponent(v);

				const Quat rot = Quat::fromAngleAxis(pi_3, v);
				Vec3 search = v.cross(axes[leastSigAxis]).normalize();

				for (UInt8 i = 0; i < 6; ++i)
				{
					csoSupport(a, b, search, outPolytope.vertices[2], outPolytope.verticesA[2], outPolytope.verticesB[2]);

					if (outPolytope.vertices[2].lengthSq() > k_epa_epsilonSq)
					{
						break;
					}

					search = rot * search;
				}
			}
			[[fallthrough]]; // end segment to triangle
		case 3:
			{
				const Vec3 u = outPolytope.vertices[1] - outPolytope.vertices[0];
				const Vec3 v = outPolytope.vertices[2] - outPolytope.vertices[0];
				Vec3 search = u.cross(v).normalize();

				csoSupport(a, b, search, outPolytope.vertices[3], outPolytope.verticesA[3], outPolytope.verticesB[3]);

				if (outPolytope.vertices[3].lengthSq() < k_epa_epsilonSq)
				{
					search = -search;
					csoSupport(a, b, search, outPolytope.vertices[3], outPolytope.verticesA[3], outPolytope.verticesB[3]);
				}
			}
			break; //end triangle to tetrahedron
		default:
			break;
		}

		UInt16 nearestTriIdx;
		Float nearestLenSq;
		Vec3 nearest = outPolytope.nearest(nearestLenSq, nearestTriIdx);

		const UInt8 MAX_ITERS = 16;
		for (UInt8 i = 0; i < MAX_ITERS; ++i)
		{
			const Vec3 search = outPolytope.normals[nearestTriIdx].normalized();

			Vec3 support, supportA, supportB;
			csoSupport(a, b, search, support, supportA, supportB);

			outPolytope.expand(support, supportA, supportB, nearestTriIdx);

			UInt16 idx;
			Float lenSq;
			Vec3 point = outPolytope.nearest(lenSq, idx);

			const bool closeEnough = Math::approx(nearestLenSq, lenSq, k_epa_epsilonSq);

			nearest = point;
			nearestLenSq = lenSq;
			nearestTriIdx = idx;

			if (closeEnough)
			{
				break;
			}
		}

		const UInt16 tidx = nearestTriIdx * 3ui16;
		const UInt16 ia = outPolytope.tris[tidx];
		const UInt16 ib = outPolytope.tris[tidx + 1ui16];
		const UInt16 ic = outPolytope.tris[tidx + 2ui16];

		const Vec3 bary = GeomUtil::barycentric(
			nearest,
			outPolytope.vertices[ia],
			outPolytope.vertices[ib],
			outPolytope.vertices[ic]);

		outContact.pointA = Body::pointToLocal(a.body(), a.pointToWorld(
			outPolytope.verticesA[ia] * bary.x
			+ outPolytope.verticesA[ib] * bary.y
			+ outPolytope.verticesA[ic] * bary.z));

		outContact.pointB = Body::pointToLocal(b.body(), b.pointToWorld(
			outPolytope.verticesB[ia] * bary.x
			+ outPolytope.verticesB[ib] * bary.y
			+ outPolytope.verticesB[ic] * bary.z));


		outContact.depth = Math::sqrt(nearestLenSq);
		outContact.normal = nearest / -outContact.depth;
		
	}
} // namespace Positional
