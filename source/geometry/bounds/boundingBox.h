#pragma once

#include <float.h>
#include <math/algebra/Point/Point3.h>
#include <math/algebra/Vector/Vector3.h>
#include <algorithm>

namespace RenderLib {
namespace Geometry {

	class BoundingBox {
	public:

		RenderLib::Math::Point3f pMin, pMax;


		BoundingBox() {
			using namespace RenderLib::Math;
			pMin = Point3f( FLT_MAX,  FLT_MAX,  FLT_MAX);
			pMax = Point3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		}
		BoundingBox(const RenderLib::Math::Point3f &p) : pMin(p), pMax(p) { }
		BoundingBox(const RenderLib::Math::Point3f &p1, const RenderLib::Math::Point3f &p2) {
			using namespace RenderLib::Math;
			pMin = Point3f(std::min(p1.x, p2.x),
						   std::min(p1.y, p2.y),
						   std::min(p1.z, p2.z));
			pMax = Point3f(std::max(p1.x, p2.x),
						   std::max(p1.y, p2.y),
						   std::max(p1.z, p2.z));
		}
		void expand( const RenderLib::Math::Point3f& p ) {
			for( int i = 0; i < 3; i++ ) {
				pMin[ i ] = std::min( pMin[ i ], p[ i ] );
				pMax[ i ] = std::max( pMax[ i ], p[ i ] );
			}
		}
		void expand( const RenderLib::Math::Point3d& p ) {
			for( int i = 0; i < 3; i++ ) {
				pMin[ i ] = std::min( pMin[ i ], (float)p[ i ] );
				pMax[ i ] = std::max( pMax[ i ], (float)p[ i ] );
			}
		}
		void expand( const BoundingBox& b ) {
			for( int i = 0; i < 3; i++ ) {
				pMin[ i ] = std::min( pMin[ i ], b.pMin[ i ] );
				pMax[ i ] = std::max( pMax[ i ], b.pMax[ i ] );
			}
		}
		bool overlaps(const BoundingBox &b) const {
			bool x = (pMax.x >= b.pMin.x) && (pMin.x <= b.pMax.x);
			bool y = (pMax.y >= b.pMin.y) && (pMin.y <= b.pMax.y);
			bool z = (pMax.z >= b.pMin.z) && (pMin.z <= b.pMax.z);
			return (x && y && z);
		}
		bool inside(const RenderLib::Math::Point3f &pt) const {
			return (pt.x >= pMin.x && pt.x <= pMax.x &&
					pt.y >= pMin.y && pt.y <= pMax.y &&
					pt.z >= pMin.z && pt.z <= pMax.z);
		}
		void expand(float delta) {
			using namespace RenderLib::Math;
			pMin -= Vector3f(delta, delta, delta);
			pMax += Vector3f(delta, delta, delta);
		}
		float volume() const {
			using namespace RenderLib::Math;
			Vector3f d = pMax - pMin;
			return d.x * d.y * d.z;
		}
		RenderLib::Math::Vector3f extents() const {
			return pMax - pMin;
		}
		int shortestAxis() const {
			using namespace RenderLib::Math;
			Vector3f diag = pMax - pMin;
			if (diag.x < diag.y && diag.x < diag.z)
				return 0;
			else if (diag.y < diag.z)
				return 1;
			else
				return 2;
		}
		int longestAxis() const {
			using namespace RenderLib::Math;
			Vector3f diag = pMax - pMin;
			if (diag.x > diag.y && diag.x > diag.z)
				return 0;
			else if (diag.y > diag.z)
				return 1;
			else
				return 2;
		}

		static BoundingBox getUnion(const BoundingBox &b, const BoundingBox &b2) 
		{
			BoundingBox res;
			res.pMin.x = std::min(b.pMin.x, b2.pMin.x);
			res.pMin.y = std::min(b.pMin.y, b2.pMin.y);
			res.pMin.z = std::min(b.pMin.z, b2.pMin.z);
			res.pMax.x = std::max(b.pMax.x, b2.pMax.x);
			res.pMax.y = std::max(b.pMax.y, b2.pMax.y);
			res.pMax.z = std::max(b.pMax.z, b2.pMax.z);
			return res;
		}
		void boundingSphere(RenderLib::Math::Point3f *c, float *rad) const 
		{
			using namespace RenderLib::Math;
			*c = ( pMin + pMax ) * 0.5f;
			*rad = Point3f::distance(*c, pMax);
		}

		RenderLib::Math::Point3f& max() {
			return pMax;
		}

		const RenderLib::Math::Point3f& max() const {
			return pMax;
		}

		RenderLib::Math::Point3f& min() {
			return pMin;
		}

		const RenderLib::Math::Point3f& mins() const {
			return pMin;
		}

		float surfaceArea() const {
			using namespace RenderLib::Math;
			const Vector3f s = extents();
			return 2.0f * ( s.x * s.y + s.x * s.z + s.y * s.z );
		}

		RenderLib::Math::Point3f center() const {
			using namespace RenderLib::Math;
			return Point3f(	( pMin[0] + pMax[0] ) * 0.5f, 
							( pMin[1] + pMax[1] ) * 0.5f, 
							( pMin[2] + pMax[2] ) * 0.5f );
		}
	};
}
}