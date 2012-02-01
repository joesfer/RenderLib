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

#include <geometry/tessellation/delaunay/delaunay2D.h>
#include <geometry/intersection/intersection.h>

namespace RenderLib {
namespace Geometry {
namespace Delaunay2D {

	bool IsTriOutside( RenderLib::Math::Vector2f centroid, 
					   const float diagonal, 
					   const LIST( int ) &edges, 
					   const LIST( RenderLib::Math::Vector2f )& vertices, 
					   int triIdx ) {

			using namespace RenderLib::Geometry;
			using namespace RenderLib::Math;
			
			(triIdx); // avoid 'unreferenced parameter' warning. This is only used for debugging

#define RETRIALS 4
			bool outside[ RETRIALS ];
			for( int retrials = 0; retrials < RETRIALS; retrials++ ) {
				Vector2f other = centroid;
				switch( retrials ) {
				case 0:
					other += Vector2f( 1, 0 ) * diagonal;
					break;
				case 1:
					other += Vector2f( -1, 0 ) * diagonal;
					break;
				case 2:
					other += Vector2f( 0, 1 ) * diagonal;
					break;
				case 3:
					other += Vector2f( 0, -1 ) * diagonal;
					break;
				default:
					break;
				}
				int intersections = 0;						

				for( size_t j = 0; j < edges.size(); j += 2 ) {
					if ( edges[ j ] == edges[ j + 1 ] || edges[ j ] < 0 || edges[ j + 1 ] < 0 ) {
						continue;
					}
					if ( SegmentIntersect( centroid, other, vertices[ edges[ j ] ], vertices[ edges[ j + 1 ] ], SEGMENT_INTERSECTION_EPSILON ) ) {
						intersections ++;
					}
				}

				outside[ retrials ] = ( intersections % 2 ) == 0;

#if _DEBUG && 0
				if ( outside[ retrials ] ) {
					MString cmd = CoreLib::varArgsStr<1024>( "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`; rename $c centroid_%d;", centroid.x, centroid.y, other.x, other.y, triIdx );
					MGlobal::executeCommand( cmd );
					for( size_t j = 0; j < edges.size(); j += 2 ) {
						if ( edges[ j ] == edges[ j + 1 ] || edges[ j ] < 0 || edges[ j + 1 ] < 0 ) {
							continue;
						}
						if ( SegmentIntersect( centroid, other, vertices[ edges[ j ] ], vertices[ edges[ j + 1 ] ], SEGMENT_INTERSECTION_EPSILON ) ) {
							Vector2f a = vertices[ edges[ j ] ];
							Vector2f b = vertices[ edges[ j + 1 ] ];
							MString cmd = CoreLib::varArgsStr<1024>( "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`; rename $c isectEdge%d_%d_%d_%d;", a.x, a.y, b.x, b.y, triIdx, j, edges[ j ], edges[ j + 1 ] );
							MGlobal::executeCommand( cmd );
						}
					}
				}
#endif


				// TODO optimize and bail out if the last > RETRIAL/2 tests gave the same result (either positive or negative)
			}

			int nPositive = 0, nNegative = 0;
			for( int j  = 0; j < RETRIALS; j++ ) {
				if ( outside[ j ] ) {
					nPositive ++;
				} else {
					nNegative ++;
				}
			}

			return nPositive >= nNegative;
		}
} // namespace Delaunay2D
} // namespace Geometry
} // namespace RenderLib