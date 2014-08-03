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

#include <geometry/intersection/intersection.h>
#include <geometry/utils.h>
#include <math/algebra/vector/vector2.h>
#include <raytracing/primitives/sphere.h>

namespace RenderLib {
namespace Geometry {

	bool segmentIntersect(  const RenderLib::Math::Vector2f A, const RenderLib::Math::Vector2f B, 
							const RenderLib::Math::Vector2f C, const RenderLib::Math::Vector2f D, 
							const float /*epsilon*/ ) {

#if 0 // using parametric equations
			const float denom = ( B.y - A.y ) * ( D.x - C.x ) - ( B.x - A.x ) * ( D.y - C.y );
			if ( fabsf( denom ) < 1e-4f ) { // segments are parallel
				return false;
			}
			const float r = -( ( D.x - C.x ) * ( A.y - C.y ) + ( D.y - C.y ) * C.x - A.x * ( D.y - C.y ) ) / denom;	
			const float s = -( (B.x - A.x ) * ( A.y - C.y ) + ( B.y - A.y ) * C.x - ( B.y - A.y ) * A.x ) / denom;
			const float dist2AB = r > 0.5f ? ( B - A ).Length() * ( 1.0f - r ) : ( B - A ).Length() * r;
			const float dist2CD = s > 0.5f ? ( D - C ).Length() * ( 1.0f - s ) : ( D - C ).Length() * s;
			return dist2AB > epsilon 
				&& dist2CD > epsilon;
#else	// using triangle areas (Real Time Collision Detection book)
			float a1 = signed2DTriangleArea( A, B, D ); // compute winding of ABD [ + or - ]
			float a2 = signed2DTriangleArea( A, B, C ); // to intersect, must have sign opposite of a1

			// if c and d are on different sides of AB, areas have different signs
			if ( a1 != 0.0f && a2 != 0.0f && a1 * a2 < 0.0f ) {
				// Compute signs of a and b with respect to segment cd
				float a3 = signed2DTriangleArea( C, D, A ); // Compute winding of cda [ + or - ]
				// since area is constant a1 - a2 = a3 - a4, or a4 = a3 + a2 - a1
				//float a4 = signed2DTriangleArea( C, D, B );
				float a4 = a3 + a2 - a1;
				// Points a and b on different sides of cd if areas have different signs
				if ( a3 * a4 < 0.0f ) {
					// Segments intersect. Find intersection poitn along L(t) = a + t * ( b - a )
					// t = a3 / ( a3 - a4 );
					// p = a + t ( b - a );
					return true;
				}
			}
			// segments not intersecting (or collinear)
			return false;
#endif

	}
}
}
