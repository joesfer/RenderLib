/*
	================================================================================
	This software is released under the LGPL-3.0 license: http://www.opensource.org/licenses/lgpl-3.0.html

	Copyright (c) 2012, Jose Esteve. http://www.joesfer.com

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 3.0 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
	================================================================================
*/
#include "geometry/bounds/boundingBox.h"
#include <float.h>
#include <algorithm>
#include <math/algebra/vector/vector3.h>

namespace RenderLib {
namespace Geometry {
		using namespace ::RenderLib::Math;
		BoundingBox::BoundingBox() {
			pMin = Point3f( FLT_MAX,  FLT_MAX,  FLT_MAX);
			pMax = Point3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		}
		BoundingBox::BoundingBox(const ::RenderLib::Math::Point3f &p) : pMin(p), pMax(p) { }
		BoundingBox::BoundingBox(const ::RenderLib::Math::Point3f &p1, const ::RenderLib::Math::Point3f &p2) {
			using namespace ::RenderLib::Math;
			pMin = Point3f(std::min(p1.x, p2.x),
						   std::min(p1.y, p2.y),
						   std::min(p1.z, p2.z));
			pMax = Point3f(std::max(p1.x, p2.x),
						   std::max(p1.y, p2.y),
						   std::max(p1.z, p2.z));
		}

		void BoundingBox::expand( const ::RenderLib::Math::Point3f& p ) {
			for( int i = 0; i < 3; i++ ) {
				pMin[ i ] = std::min( pMin[ i ], p[ i ] );
				pMax[ i ] = std::max( pMax[ i ], p[ i ] );
			}
		}
		
		void BoundingBox::expand( const ::RenderLib::Math::Point3d& p ) {
			for( int i = 0; i < 3; i++ ) {
				const float fP_i = float(p[i]);
				pMin[ i ] = std::min( pMin[ i ], fP_i );
				pMax[ i ] = std::max( pMax[ i ], fP_i );
			}
		}

		void BoundingBox::expand( const BoundingBox& b ) {
			for( int i = 0; i < 3; i++ ) {
				pMin[ i ] = std::min( pMin[ i ], b.pMin[ i ] );
				pMax[ i ] = std::max( pMax[ i ], b.pMax[ i ] );
			}
		}
		void BoundingBox::expand(float delta) {
			pMin -= Vector3f(delta, delta, delta);
			pMax += Vector3f(delta, delta, delta);
		}
		bool BoundingBox::overlaps(const BoundingBox &b) const {
			bool x = (pMax.x >= b.pMin.x) && (pMin.x <= b.pMax.x);
			bool y = (pMax.y >= b.pMin.y) && (pMin.y <= b.pMax.y);
			bool z = (pMax.z >= b.pMin.z) && (pMin.z <= b.pMax.z);
			return (x && y && z);
		}
		bool BoundingBox::inside(const ::RenderLib::Math::Point3f &pt) const {
			return (pt.x >= pMin.x && pt.x <= pMax.x &&
					pt.y >= pMin.y && pt.y <= pMax.y &&
					pt.z >= pMin.z && pt.z <= pMax.z);
		}
		float BoundingBox::volume() const {
			Vector3f d = pMax - pMin;
			return d.x * d.y * d.z;
		}
		Vector3f BoundingBox::extents() const {
			return pMax - pMin;
		}
		int BoundingBox::shortestAxis() const {
			Vector3f diag = pMax - pMin;
			if (diag.x < diag.y && diag.x < diag.z)
				return 0;
			else if (diag.y < diag.z)
				return 1;
			else
				return 2;
		}
		int BoundingBox::longestAxis() const {
			Vector3f diag = pMax - pMin;
			if (diag.x > diag.y && diag.x > diag.z)
				return 0;
			else if (diag.y > diag.z)
				return 1;
			else
				return 2;
		}

		/*static*/ BoundingBox BoundingBox::getUnion(const BoundingBox &b, const BoundingBox &b2) 
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
		void BoundingBox::boundingSphere(RenderLib::Math::Point3f& c, float& rad) const 
		{
			c = ( pMin + pMax ) * 0.5f;
			rad = Point3f::distance(c, pMax);
		}

		const Point3f& BoundingBox::max() const {
			return pMax;
		}

		const Point3f& BoundingBox::min() const {
			return pMin;
		}

		Point3f& BoundingBox::max() {
			return pMax;
		}

		Point3f& BoundingBox::min() {
			return pMin;
		}

		float BoundingBox::surfaceArea() const {
			const Vector3f s = extents();
			return 2.0f * ( s.x * s.y + s.x * s.z + s.y * s.z );
		}

		Point3f BoundingBox::center() const {
			return Point3f(	( pMin[0] + pMax[0] ) * 0.5f, 
							( pMin[1] + pMax[1] ) * 0.5f, 
							( pMin[2] + pMax[2] ) * 0.5f );
		}

}
}