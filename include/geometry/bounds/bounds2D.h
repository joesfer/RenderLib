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

#pragma once

#include <float.h>
#include <math.h>
#include <math/algebra/point/point2.h>
#include <math/algebra/vector/vector2.h>

namespace RenderLib {
namespace Geometry {

	class Bounds2D {
	public:

		Bounds2D() {
			pMin = RenderLib::Math::Point2f( FLT_MAX,  FLT_MAX );
			pMax = RenderLib::Math::Point2f(-FLT_MAX, -FLT_MAX );
		}
		Bounds2D(const RenderLib::Math::Point2f &p) : pMin(p), pMax(p) { }
		Bounds2D(const RenderLib::Math::Point2f &p1, const RenderLib::Math::Point2f &p2) {
			pMin = RenderLib::Math::Point2f( std::min(p1.x, p2.x), std::min(p1.y, p2.y) );
			pMax = RenderLib::Math::Point2f( std::max(p1.x, p2.x), std::max(p1.y, p2.y) );
		}
		void expand( const RenderLib::Math::Point2f& p );
		void expand( const RenderLib::Math::Point2d& p );
		void expand( const RenderLib::Math::Vector2f& p );
		void expand( const Bounds2D& b );
		void expand(float delta);

		bool overlaps(const Bounds2D &b) const;

		bool inside(const RenderLib::Math::Point2f &pt) const {
			return (pt.x >= pMin.x && pt.x <= pMax.x &&
					pt.y >= pMin.y && pt.y <= pMax.y );
		}
	
		RenderLib::Math::Vector2f extents() const;
		int shortestAxis() const;
		int longestAxis() const;

		static Bounds2D getUnion(const Bounds2D &b, const RenderLib::Math::Point2f &p);
		static Bounds2D getUnion(const Bounds2D &b, const Bounds2D &b2);
		
		const RenderLib::Math::Point2f& max() const { return pMax; }
		const RenderLib::Math::Point2f& min() const { return pMin; }
		RenderLib::Math::Point2f& max() { return pMax; }
		RenderLib::Math::Point2f& min() { return pMin; }
		
		void boundingCircunference(RenderLib::Math::Point2f *c, float *rad) const;
		float surfaceArea() const;
		RenderLib::Math::Point2f center() const;

		private:

			RenderLib::Math::Point2f pMin, pMax;

};

} // namespace Geometry
} // namespace RenderLib