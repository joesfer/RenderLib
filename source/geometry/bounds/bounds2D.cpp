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

#include <geometry/bounds/bounds2D.h>

namespace RenderLib {
namespace Geometry {

	RenderLib::Math::Vector2f Bounds2D::extents() const {
		return pMax - pMin;
	}
	int Bounds2D::shortestAxis() const {
		const RenderLib::Math::Vector2f diag = pMax - pMin;
		return diag.x <= diag.y ? 0 : 1;
	}
	int Bounds2D::longestAxis() const {
		const RenderLib::Math::Vector2f diag = pMax - pMin;
		return diag.x >= diag.y ? 0 : 1;
	}

	/*static*/ Bounds2D Bounds2D::getUnion(const Bounds2D &b, const RenderLib::Math::Point2f &p) 
	{
		Bounds2D res = b;
		res.pMin.x = std::min(b.pMin.x, p.x);
		res.pMin.y = std::min(b.pMin.y, p.y);
		res.pMax.x = std::max(b.pMax.x, p.x);
		res.pMax.y = std::max(b.pMax.y, p.y);
		return res;
	}
	/*static*/ Bounds2D Bounds2D::getUnion(const Bounds2D &b, const Bounds2D &b2) 
	{
		Bounds2D res;
		res.pMin.x = std::min(b.pMin.x, b2.pMin.x);
		res.pMin.y = std::min(b.pMin.y, b2.pMin.y);
		res.pMax.x = std::max(b.pMax.x, b2.pMax.x);
		res.pMax.y = std::max(b.pMax.y, b2.pMax.y);
		return res;
	}

	void Bounds2D::expand( const RenderLib::Math::Point2f& p ) {
		for( int i = 0; i < 2; i++ ) {
			pMin[ i ] = std::min( pMin[ i ], p[ i ] );
			pMax[ i ] = std::max( pMax[ i ], p[ i ] );
		}
	}
	void Bounds2D::expand( const RenderLib::Math::Point2d& p ) {
		for( int i = 0; i < 2; i++ ) {
			pMin[ i ] = std::min( pMin[ i ], (float)p[ i ] );
			pMax[ i ] = std::max( pMax[ i ], (float)p[ i ] );
		}
	}
	void Bounds2D::expand( const RenderLib::Math::Vector2f& p ) {
		for( int i = 0; i < 2; i++ ) {
			pMin[ i ] = std::min( pMin[ i ], (float)p[ i ] );
			pMax[ i ] = std::max( pMax[ i ], (float)p[ i ] );
		}
	}
	void Bounds2D::expand( const Bounds2D& b ) {
		for( int i = 0; i < 2; i++ ) {
			pMin[ i ] = std::min( pMin[ i ], b.pMin[ i ] );
			pMax[ i ] = std::max( pMax[ i ], b.pMax[ i ] );
		}
	}
	void Bounds2D::expand(float delta) {
		pMin -= RenderLib::Math::Vector2f(delta, delta );
		pMax += RenderLib::Math::Vector2f(delta, delta );
	}

	void Bounds2D::boundingCircunference(RenderLib::Math::Point2f *c, float *rad) const 
	{
		*c = ( pMin + pMax ) * 0.5f;
		*rad = RenderLib::Math::Point2f::distance(*c, pMax);
	}

	float Bounds2D::surfaceArea() const {
		RenderLib::Math::Vector2f s = extents();
		return ( s.x * s.y );
	}

	RenderLib::Math::Point2f Bounds2D::center() const {
		return RenderLib::Math::Point2f(( pMin[0] + pMax[0] ) * 0.5f, 
										( pMin[1] + pMax[1] ) * 0.5f );
	}

} // namespace Geometry
} // namespace RenderLib