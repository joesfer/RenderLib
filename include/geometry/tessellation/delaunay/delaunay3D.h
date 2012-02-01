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

#include <CoreLib.h>
#include <math/algebra/vector/vector3.h>
#include <math/algebra/point/point3.h>
#include <geometry/utils.h>
#include <assert.h>
#include <memory.h>
#include <stack>

#define LEAVE_CONTAINING_TETRAHEDRON 1

namespace RenderLib {
namespace Geometry {
namespace Delaunay3D {

	typedef double REAL;
	typedef RenderLib::Math::Point3<REAL> Point;	

	//////////////////////////////////////////////////////////////////////////
	// struct tetrahedron_t
	//////////////////////////////////////////////////////////////////////////

	struct tetrahedron_t {
		int					v[ 4 ];	// vertex indices
		int			neighbors[ 4 ]; // tetrahedron index adjacent to the i-th face
		int			face[ 4 ][ 3 ]; // vertex indices for each face

		tetrahedron_t() {
			MarkInvalid();			
		}

		tetrahedron_t( const tetrahedron_t& other ) {
			memcpy( v, other.v, 4 * sizeof( int ) );
			memcpy( neighbors, other.neighbors, 4 * sizeof( int ) );
			memcpy( face, other.face, 3 * 4 * sizeof( int ) );
		}

		void MarkInvalid() {
			for( int i = 0; i < 4; i++ ) {
				v[ i ] = -1;
				neighbors[ i ] = -1;
			}
			face[ 0 ][ 0 ] = 0; face[ 0 ][ 1 ] = 1; face[ 0 ][ 2 ] = 2;
			face[ 1 ][ 0 ] = 0; face[ 1 ][ 1 ] = 3; face[ 1 ][ 2 ] = 1;
			face[ 2 ][ 0 ] = 1; face[ 2 ][ 1 ] = 3; face[ 2 ][ 2 ] = 2;
			face[ 3 ][ 0 ] = 2; face[ 3 ][ 1 ] = 3; face[ 3 ][ 2 ] = 0;
		}
		
		bool IsValid() const {
			for( int i = 0; i < 4; i++ ) {
				if ( v[ i ] < 0 ) {
					return false;
				}
			}
			return true;
		}
		
		bool ContainsVertex( const int vert ) const {
			return v[ 0 ] == vert || v[ 1 ] == vert || v[ 2 ] == vert || v[ 3 ] == vert;
		}

		void GetFaceVertices( const int f, int& a, int& b, int& c ) const {
			assert( f >= 0 && f < 4 );
			assert( face[f][0] >= 0 && face[f][0] < 4 );
			assert( face[f][1] >= 0 && face[f][1] < 4 );
			assert( face[f][2] >= 0 && face[f][2] < 4 );
			a = v[face[f][0]]; b = v[face[f][1]]; c = v[face[f][2]];
		}

		REAL GetFaceArea( const int f, const CoreLib::List< Point >& vertices ) const {
			using namespace RenderLib::Geometry;
			switch( f ) {
				case 0:
					return TriangleArea( vertices[ v[0] ], vertices[ v[1] ], vertices[ v[2] ] );
				case 1:
					return TriangleArea( vertices[ v[0] ], vertices[ v[3] ], vertices[ v[1] ] );
				case 2:
					return TriangleArea( vertices[ v[1] ], vertices[ v[3] ], vertices[ v[2] ] );
				case 3:
					return TriangleArea( vertices[ v[2] ], vertices[ v[3] ], vertices[ v[0] ] );
				default: 
					assert( false );
					return -1;
			}
		}

		int GetFaceFromVertices( const int a, const int b, const int c ) const {			
			for( int i = 0; i < 4; i++ ) {
				int check = 0;
				for( int j = 0; j < 3; j++ ) {
					if ( v[ face[ i ][ j ] ] == a || v[ face[ i ][ j ] ] == b || v[ face[ i ][ j ] ] == c ) {
						check |= ( 1 << j );						
					}
				}
				if ( check == 7 /* 111 */ ) {
					return i;
				}
			}
						
			return -1;
		}

		int GetVertexOutsideFace( int f ) const {
			assert( f >= 0 && f < 4 );
			int check = 0;
			for( int i = 0; i < 3; i++ ) {
				check |= ( 1 << face[ f ][ i ] );
			}
			switch( check ) {
				case 14: // 1110
					return v[0];
				case 13: // 1101
					return v[1];
				case 11: // 1011
					return v[2];
				case 7: // 0111
					return v[3];
				default:
					assert( false );
					return -1;
			}
		}
	

		// We use this function to check whether two faces sharing the same vertices are reversed
		// without needing to calculate the normal
		bool SameWinding( const int v1[3], const int v2[3] ) const {
			int offset2;
			for( offset2 = 0; offset2 < 3; offset2++ ) {
				if ( v2[ offset2 ] == v1[ 0 ] ) {
					break;
				}
			}
			if ( offset2 == 3 ) { 
				// sequences are not even the same
				return false;
			}
			for( int offset1 = 1; offset1 < 3; offset1++ ) {
				offset2 = ( offset2 + 1 ) % 3;
				if ( v1[ offset1 ] != v2[ offset2 ] ) {
					return false;
				}
			}
			return true;
		}

		int SharedFace( const tetrahedron_t& other, bool reversed ) const {
			int verts[ 3 ];
			int verts2[3];
			for( int i = 0; i < 4; i++ ) {
				other.GetFaceVertices( i, verts[ 0 ], verts[ 1 ], verts[ 2 ] );
				int face = GetFaceFromVertices( verts[ 0 ], verts[ 2 ], verts[ 1 ] ); // reverse the face order as they're confronted
				if ( face >= 0 ) {
					GetFaceVertices( face, verts2[ 0 ], verts2[ 1 ], verts2[ 2 ] );

					if ( reversed &&  // tetrahedra are adjacent on each side of the adjacent face, so the face is declared on reverse order for each tetrahedron 
						 SameWinding( verts, verts2 ) ) {
							face = -1;					
					} 
					if ( !reversed && // the tetrahedra overlap on the same space (during flip 44 for example) so the shared faces have the same orientation
						!SameWinding( verts, verts2 ) ) {
							face = -1;					
					}
					if ( face >= 0 ) {
						return face;
					}
				}
			}

			return -1;
		}

		bool AdjacentTo( const tetrahedron_t& other ) const {
			return SharedFace( other, true ) >= 0;
		}

		bool CheckNeighbors( const int thisIndex, const CoreLib::List< tetrahedron_t >& tetrahedra, const CoreLib::List< Point >& vertices ) const {
			for( int i = 0; i < 4; i++ ) {
				if ( neighbors[ i ] >= 0 ) {
					const tetrahedron_t& neighbor = tetrahedra[ neighbors[ i ] ];
					if ( !neighbor.IsValid() ) {
						return false;
					}

					const int sf = neighbor.SharedFace( *this, true );
					if ( sf < 0 ) {
						return false;
					}

					if ( SameOrientation( i, neighbor, sf, vertices ) ) { // they should be reversed
						return false;
					}

					if ( neighbor.neighbors[ sf ] != thisIndex ) {
						return false;
					}
				}
			}
			return true;
		}

		void Destroy( CoreLib::List< tetrahedron_t >& tetrahedra ) {			
			// unlink neighbors
			for( int i = 0; i < 4; i++ ) {
				if ( neighbors[ i ] >= 0 ) {
					tetrahedron_t& n = tetrahedra[ neighbors[ i ] ];
					if ( !n.IsValid() ) {
						continue;
					}
					const int sf = n.SharedFace( *this, true );
					n.neighbors[ sf ] = -1;
				}
			}

			MarkInvalid();
		}

		// checks whether 'face' has the same orientation than 'otherFace' (normals point toward the same direction)
		bool SameOrientation( const int face, const tetrahedron_t& other, const int otherFace, const CoreLib::List< Point >& vertices ) const {
			using namespace RenderLib::Math;
			int vi1[3], vi2[3];
			GetFaceVertices( face, vi1[0], vi1[1], vi1[2] );
			other.GetFaceVertices( otherFace, vi2[0], vi2[1], vi2[2] );
			Vector3<REAL> n1 = Vector3<REAL>::cross( vertices[vi1[1]] - vertices[vi1[0]], vertices[vi1[2]] - vertices[vi1[0]] );
			Vector3<REAL> n2 = Vector3<REAL>::cross( vertices[vi2[1]] - vertices[vi2[0]], vertices[vi2[2]] - vertices[vi2[0]] );
			return Vector3<REAL>::dot(n1, n2) > (REAL)0;
		}

		void ReverseFace( const int f ) {
			const int v0 = face[ f ][ 0 ];
			face[ f ][ 0 ] = face[ f ][ 2 ];
			face[ f ][ 2 ] = v0;
		}

		bool IsFlat( const CoreLib::List< Point >& vertices ) const;

		// ensure the tetrahedron centroid lies behind every face
		void FixFaceOrientations( const CoreLib::List< Point >& vertices );
		
		bool CheckFaceOrientations( const CoreLib::List< Point >& vertices ) const;	
	};

	//////////////////////////////////////////////////////////////////////////
	// class Delaunay3D
	//
	// 3D Delaunay tessellation (tetrahedralization)
	//////////////////////////////////////////////////////////////////////////
	
	class Delaunay3D {
		friend struct tetrahedron_t;
	public:
#if LEAVE_CONTAINING_TETRAHEDRON 
		bool Tetrahedralize( CoreLib::List< Point >& points, 
							 CoreLib::List< tetrahedron_t >& tetrahedra );
#else
		bool Tetrahedralize( const CoreLib::List< Point >& points, 
							 CoreLib::List< tetrahedron_t >& tetrahedra );
#endif

	private:
	
		// Predicates //////////////////////////////////////////////////////////////////////////

		// Determines whether p is above the plane ( > 0 ), below ( < 0 ) or on the plane ( = 0 )
		static REAL Orient(  const Point& a, 
							 const Point& b, 
							 const Point& c, 
							 const Point& p );

		// returns > 0 if p is inside the sphere described by A,B,C,D, < 0 outside, = 0 on the sphere
		static REAL InSphere( const Point& a, 
							  const Point& b, 
							  const Point& c, 
							  const Point& d, 
							  const Point& p );

		// Is p inside tetrahedron t?
		static bool Inside( const Point& p, const tetrahedron_t& t, const CoreLib::List< Point >& vertices );

		// whether 4 points are coplanar
		static bool Coplanar( const Point& a, const Point& b, const Point& c, const Point& d );

		// Bistellar flips /////////////////////////////////////////////////////////////////////

		static void Flip14( const unsigned int pointIndex, const unsigned int tetrahedron, 
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[4] );

		static bool Flip23( const unsigned int tetrahedron1, const unsigned int tetrahedron2, 							
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[3] );

		static void Flip32( const unsigned int tetrahedron1, const unsigned int tetrahedron2, const unsigned int tetrahedron3, 							
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[2] );

		static void Flip44( const unsigned int tetrahedron1, const unsigned int tetrahedron2, const unsigned int tetrahedron3, const unsigned int tetrahedron4,							
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[4] );

		// Auxiliary operations ////////////////////////////////////////////////////////////////

		static void Containingtetrahedron( const RenderLib::Math::Point3f& center, const float radius, // circumsphere  
											tetrahedron_t& t, CoreLib::List< Point >& points );

		// Given a tetrahedron and a face, ensures the potential adjacent tetrahedron sharing
		// that face points to the given tetrahedron.
		static void AdjustNeighborVicinity( const int iT, const int f, CoreLib::List< tetrahedron_t >& tetrahedra );

		// returns the index of the tetrahedron containing p, or -1 if not found. 
		// It retrieves the result by searching from a given sourceT tetrahedron index. 
		static int Walk( const Point& p, const int sourceT, const CoreLib::List< Point >& vertices, const CoreLib::List< tetrahedron_t >& tetrahedra );

		// Flips two given tetrahedra: T and Ta, which are non-Delaunay, 
		// into a Delaunay configuration using bistellar flips
		static void Flip( const int T, const int Ta, const int p,
						const CoreLib::List< Point >& vertices,
						CoreLib::List< tetrahedron_t >& tetrahedra,
						std::stack< unsigned int >& needTesting );

		static void InsertOnePoint( const CoreLib::List< Point >& vertices, const int pointIndex, 
									CoreLib::List< tetrahedron_t >& tetrahedra );

	private:

		CoreLib::List< REAL > tempVertices;

	};

} // namespace Delaunay3D
} // namespace Geometry
} // namespace RenderLib
