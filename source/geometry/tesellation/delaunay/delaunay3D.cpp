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

#include <math/algebra/matrix/matrix4.h>
#include <math/algebra/matrix/matrix5.h>
#include <math/algebra/vector/vector3.h>
#include <geometry/tessellation/delaunay/delaunay3D.h>
#include <geometry/bounds/boundingBox.h>
#include <geometry/intersection/intersection.h>
#include <math.h>
#include <stack>

namespace RenderLib {
namespace Geometry {
namespace Delaunay3D {

	bool tetrahedron_t::CheckFaceOrientations( const CoreLib::List< Point >& vertices ) const {
		// check whether the tetrahedron centroid lies behind every face

		const Point& vo = vertices[ v[ 0 ] ];
		const Point& va = vertices[ v[ 1 ] ];
		const Point& vb = vertices[ v[ 2 ] ];
		const Point& vc = vertices[ v[ 3 ] ];
#if 0 // using centroid
		const Vector3<REAL> a = va - vo;
		const Vector3<REAL> b = vb - vo;
		const Vector3<REAL> c = vc - vo;
		Point C = vo + ( a + b + c ) / 4;
#elif 1 // using incenter
		const REAL a = GetFaceArea( 2, vertices );
		const REAL b = GetFaceArea( 3, vertices );
		const REAL c = GetFaceArea( 1, vertices );
		const REAL d = GetFaceArea( 0, vertices );
		const REAL t = a + b + c + d;
		Point C = vo * ( a / t ) + va * ( b / t ) + vb * ( c / t ) + vc * ( d / t);
#else // monge point
		const Vector3<REAL> a = va - vo;
		const Vector3<REAL> b = vb - vo;
		const Vector3<REAL> c = vc - vo;
		const Vector3<REAL> bCrossC = b.Cross( c );
		const Vector3<REAL> cCrossA = c.Cross( a );
		const Vector3<REAL> aCrossB = a.Cross( b );

		Point C = vo + ( a * ( ( b + c ) * bCrossC ) + b * ( ( c + a ) * cCrossA ) + c * ( ( a + b ) * aCrossB ) ) / ( a * bCrossC * 2 );

#endif
		int v2[3];
		for( int i = 0; i < 4; i++ ) {
			GetFaceVertices( i, v2[0], v2[1], v2[2] );
			/*const Point faceCentroid = ( vertices[ v2[ 0 ] ] + vertices[ v2[ 1 ] ] + vertices[ v2[ 2 ] ] ) / 3;
			Vector3<REAL> faceNormal = Vector3<REAL>::Cross( vertices[ v2[ 2 ] ] - vertices[ v2[ 0 ] ], vertices[ v2[ 1 ] ] - vertices[ v2[ 0 ] ] );
			const Vector3< REAL > fromCentroid = faceCentroid - C;
			if ( faceNormal * fromCentroid < -(REAL)0.001 ) {
				return false;
			}*/
			if( Delaunay3D::Orient( vertices[ v2[0] ], vertices[ v2[1] ], vertices[ v2[2] ], C ) > 0 ) {
				return false;
			}
		}
		return true;
	}

	/*
	================
	tetrahedron_t::IsFlat

	A tetrahedron is flat if its 4 vertices lie in the same plane
	A flat tetrahedron has no circumsphere
	================
	*/
	bool tetrahedron_t::IsFlat( const CoreLib::List< Point >& vertices ) const {
		return Delaunay3D::Coplanar( vertices[ v[ 0 ] ], vertices[ v[ 1 ] ], vertices[ v[ 2 ] ], vertices[ v[ 3 ] ] );
	}

	void tetrahedron_t::FixFaceOrientations( const CoreLib::List< Point >& vertices ) {
		using namespace RenderLib::Math;

		// check whether the tetrahedron centroid lies behind every face

		const Point& vo = vertices[ v[ 0 ] ];
		const Point& va = vertices[ v[ 1 ] ];
		const Point& vb = vertices[ v[ 2 ] ];
		const Point& vc = vertices[ v[ 3 ] ];

		if ( IsFlat( vertices ) ) {
			// the test makes no sense on flat tetrahedra as the centroid is neither inside nor outside the tetrahedron
			return;
		}
#if 1 // using centroid
		const Vector3<REAL> a = va - vo;
		const Vector3<REAL> b = vb - vo;
		const Vector3<REAL> c = vc - vo;
		Point C = vo + ( a + b + c ) / 4;
#elif 1 // using incenter
		const REAL a = GetFaceArea( 2, vertices );
		const REAL b = GetFaceArea( 3, vertices );
		const REAL c = GetFaceArea( 1, vertices );
		const REAL d = GetFaceArea( 0, vertices );
		const REAL t = a + b + c + d;
		Point C = vo * ( a / t ) + va * ( b / t ) + vb * ( c / t ) + vc * ( d / t);
#else // monge point
		const Vector3<REAL> a = va - vo;
		const Vector3<REAL> b = vb - vo;
		const Vector3<REAL> c = vc - vo;
		const Vector3<REAL> bCrossC = b.Cross( c );
		const Vector3<REAL> cCrossA = c.Cross( a );
		const Vector3<REAL> aCrossB = a.Cross( b );

		Point C = vo + ( a * ( ( b + c ) * bCrossC ) + b * ( ( c + a ) * cCrossA ) + c * ( ( a + b ) * aCrossB ) ) / ( a * bCrossC * 2 );

#endif
		int v2[3];
		for( int i = 0; i < 4; i++ ) {
			GetFaceVertices( i, v2[0], v2[1], v2[2] );
			const Vector3<REAL> faceNormal = Vector3<REAL>::normalize( Vector3<REAL>::cross( vertices[ v2[2] ] - vertices[ v2[0] ], vertices[ v2[1] ] - vertices[ v2[0] ] ) ); // we can get away not normalizing it, since we're just interested in the distance sign
			const REAL d = Vector3<REAL>::dot(faceNormal, vertices[ v2[0] ].fromOrigin());
			const REAL distToPlane = Vector3<REAL>::dot(faceNormal, C.fromOrigin()) - d;
			const REAL epsilon = 1e-1;
			if ( distToPlane > epsilon ) { // the centroid should be behind the plane
				ReverseFace( i ); 
			}
		}				
	}

#if LEAVE_CONTAINING_TETRAHEDRON 
	bool Delaunay3D::Tetrahedralize( CoreLib::List< Point >& srcPoints,
							 		 CoreLib::List< tetrahedron_t >& tetrahedra ) {
#else
	bool Delaunay3D::Tetrahedralize( const CoreLib::List< Point >& srcPoints,
		CoreLib::List< tetrahedron_t >& tetrahedra ) {
#endif
		using namespace RenderLib::Math;
		using namespace RenderLib::Geometry;

		if ( srcPoints.size() == 0 ) { 
			return false;
		}

		BoundingBox bounds;
		for( size_t i = 0; i < srcPoints.size(); i++ ) {
			bounds.expand( srcPoints[ i ] );
		}

		Point3f center;
		float radius;
		bounds.boundingSphere( center, radius );
		radius *= 2; // avoid a too tight bound since we want the containing tetrahedron's faces to wrap all the points
		if ( srcPoints.size() < 2 ) {
			// we don't have enough points to define a volume
			// so give it a fixed extra radius
			radius = 1.0;
		}
		
		// copy the source points list, we will need to add some extra
		// points while building the tessellation, but then we'll restrict
		// the resulting tetrahedra to the initial point set.
#if	LEAVE_CONTAINING_TETRAHEDRON
		CoreLib::List< Point >& pointSet = srcPoints;
#else
		CoreLib::List< Point > pointSet = srcPoints;
#endif
		const size_t numSrcPoints = srcPoints.size();

		// generate the tetrahedron from the sphere it contains (circumsphere)
		tetrahedron_t& bigT = tetrahedra.append();
		Containingtetrahedron( center, radius, bigT, pointSet );
		bigT.FixFaceOrientations( pointSet );

		tetrahedra.setGranularity( 4 * numSrcPoints );
		for( size_t i = 0; i < numSrcPoints; i++ ) {
			InsertOnePoint( pointSet, i, tetrahedra );
		}
#if _DEBUG
		// verify Delaunay condition (empty spheres) for all tetrahedra
		for( size_t i = 0; i < tetrahedra.size(); i++ ) {
			const tetrahedron_t& T = tetrahedra[ i ];
			if ( !T.IsValid() ) {
				continue;
			}
			const Point& T0 = pointSet[ T.v[ 0 ] ];
			const Point& T1 = pointSet[ T.v[ 2 ] ];
			const Point& T2 = pointSet[ T.v[ 1 ] ];
			const Point& T3 = pointSet[ T.v[ 3 ] ];
			for( size_t j = 0; j < pointSet.size(); j++ ) {
				assert( InSphere( T0, T1, T2, T3, pointSet[ j ] ) <= 0 );
			}
		}
#endif
#if !LEAVE_CONTAINING_TETRAHEDRON
		// Delete all tetrahedra containing one of the additional vertices
		// inserted for the containing tetrahedron
		for( size_t i = 0; i < tetrahedra.size(); i++ ) {
			tetrahedron_t& t = tetrahedra[ i ];
			for( size_t j = 0; j < 4; j++ ) {
				if( t.v[ j ] >= srcPoints.size() ) {
					t.Destroy( tetrahedra );
					break;
				}
			}			
		}
#endif
		return true;
	}

	//////////////////////////////////////////////////////////////////////////
	// Predicates
	//////////////////////////////////////////////////////////////////////////

	/*
	================
	Delaunay3D::Orient

	Determines whether p is above ( > 0 ) the plane defined by A,B,C, 
	below ( < 0 ) or on the plane ( = 0 )
	The test follows the left-hand rule, clockwise order = up
	================
	*/
	REAL Delaunay3D::Orient( const Point& a, const Point& b, const Point& c, const Point& p ) {
		using namespace RenderLib::Math;
		Matrix4<REAL> M( a.x, a.y, a.z, 1,
						 b.x, b.y, b.z, 1, 
						 c.x, c.y, c.z, 1, 
						 p.x, p.y, p.z, 1 );
		
		const REAL epsilon = (REAL)0.001;
		const REAL det = M.determinant();

#if _DEBUG
		{
			Vector3<REAL> n = Vector3<REAL>::cross( c - a , b - a );
			if ( n.lengthSquared() > 1e-4 ) {
				n.normalize();
				const REAL d = Vector3<REAL>::dot(n, a.fromOrigin());
				const REAL distToPlane = Vector3<REAL>::dot(n, p.fromOrigin()) - d;
				if ( det > epsilon ) {
					assert( distToPlane > 0 );
				} else if ( det < -epsilon ) {
					assert( distToPlane < 0 );
				}
			}
		}
#endif

		if( det < -epsilon || det > epsilon ) {
			return det;
		} 
		return 0;
	}

	/*
	================
	Delaunay3D::InSphere

	Returns > 0 if p is inside the sphere described by a,b,c,d, 
	< 0 outside, = 0 on the sphere
	================
	*/
	REAL Delaunay3D::InSphere( const Point& a, const Point& b, const Point& c, const Point& d, const Point& p ) {
		using namespace RenderLib::Math;
		Matrix5<REAL> M( a.x, a.y, a.z, a.x * a.x + a.y * a.y + a.z * a.z, 1, 
						 b.x, b.y, b.z, b.x * b.x + b.y * b.y + b.z * b.z, 1, 
						 c.x, c.y, c.z, c.x * c.x + c.y * c.y + c.z * c.z, 1, 
						 d.x, d.y, d.z, d.x * d.x + d.y * d.y + d.z * d.z, 1, 
						 p.x, p.y, p.z, p.x * p.x + p.y * p.y + p.z * p.z, 1 );

		const REAL epsilon = (REAL)0.0001;
		const REAL det = M.determinant();

		assert( Orient( a, b, c, d ) >= 0 );
#if _DEBUG
		{
			Point center; 
			REAL radius = 0;
			SphereFrom4Points<REAL>( a, b, c, d, center, radius );
			const REAL dist = center.distanceTo( p );

			if( det < -epsilon ) { 
				assert( dist > radius - epsilon );
			} else if ( det > epsilon ) {
				assert( dist < radius + epsilon );
			} 			
		}
#endif

		if( det < -epsilon || det > epsilon ) {
			return det;
		} 
		return 0;
	}

	/*
	================
	Delaunay3D::Inside

	Whether the point p is inside the tetrahedron t
	================
	*/
	bool Delaunay3D::Inside( const Point& p, const tetrahedron_t& t, const CoreLib::List< Point >& vertices ) {
		
		using namespace RenderLib::Math;

		int a, b, c;
		if ( t.IsFlat( vertices ) ) {
			return false;
		}

		for( int i = 0; i < 4; i++ ) {
			t.GetFaceVertices( i, a, b, c );
			//if ( Orient( vertices[ a ], vertices[ b ], vertices[ c ], p ) > 0 ) {
			//	return false;
			//}
			const Vector3<REAL> n = Vector3<REAL>::normalize( Vector3<REAL>::cross( vertices[ c ] - vertices[ a ], vertices[ b ] - vertices[ a ] ) );
			const REAL d = Vector3<REAL>::dot(n, vertices[ a ].fromOrigin());
			const REAL distToPlane = Vector3<REAL>::dot(n, p.fromOrigin()) - d;
			if ( distToPlane > 1e-4 ) {
				return false;
			}
		}
		return true;
	}	

	/*
	================
	Delaunay3D::Coplanar

	Whether 4 points are coplanar
	================
	*/
	bool Delaunay3D::Coplanar( const Point& a, const Point& b, const Point& c, const Point& d ) {
		return Orient( a, b, c, d ) == 0;
	}

	//////////////////////////////////////////////////////////////////////////
	// Bistellar Flips
	//////////////////////////////////////////////////////////////////////////

	/*
	================
	Delaunay3D::Flip14

	Bistellar Flip 1->4 : inserts the point given by pointIndex into 
	the tetrahedron splitting it into 4 new adjacent tetrahedrons
	================
	*/
	void Delaunay3D::Flip14( const unsigned int pointIndex, const unsigned int tetrahedron, 
							 CoreLib::List< tetrahedron_t >& tetrahedra,
							 const CoreLib::List< Point >& vertices,
							 unsigned int resultingTetrahedra[4] ) {

		tetrahedron_t srcT = tetrahedra[ tetrahedron ];
		assert( srcT.CheckFaceOrientations( vertices ) );

		assert( srcT.CheckNeighbors( tetrahedron, tetrahedra, vertices ) );

#if _DEBUG
		for( int i = 0; i < 4; i++ ) {
			assert( srcT.v[ i ] != pointIndex );
		}
#endif

		assert( Inside( vertices[ pointIndex ], srcT, vertices ) );

		tetrahedra.preAllocate( tetrahedra.size() + 3 ); // ensure the list is not resized while we're holding references to its elements

		const unsigned int iResT1 = tetrahedron; // reuse the source tetrahedron slot
		tetrahedron_t& resT1 = tetrahedra[ iResT1 ];
		resT1.Destroy( tetrahedra );
		const unsigned int iResT2 = tetrahedra.size(); 
		tetrahedron_t& resT2 = tetrahedra.append();
		const unsigned int iResT3 = tetrahedra.size(); 
		tetrahedron_t& resT3 = tetrahedra.append();
		const unsigned int iResT4 = tetrahedra.size(); 
		tetrahedron_t& resT4 = tetrahedra.append();

		assert( !resT1.IsValid() );
		assert( !resT2.IsValid() );
		assert( !resT3.IsValid() );
		assert( !resT4.IsValid() );

		resultingTetrahedra[ 0 ] = iResT1;
		resultingTetrahedra[ 1 ] = iResT2;
		resultingTetrahedra[ 2 ] = iResT3;
		resultingTetrahedra[ 3 ] = iResT4;

		// res1
		srcT.GetFaceVertices( 0, resT1.v[0], resT1.v[1], resT1.v[2] );
		resT1.v[ 3 ] = pointIndex;
		resT1.FixFaceOrientations( vertices );
		assert( srcT.SameOrientation( 0, resT1, 0, vertices ) );

		// res2
		srcT.GetFaceVertices( 1, resT2.v[0], resT2.v[1], resT2.v[2] );
		resT2.v[ 3 ] = pointIndex;
		resT2.FixFaceOrientations( vertices );		
		assert( srcT.SameOrientation( 1, resT2, 0, vertices ) );
	
		// res3
		srcT.GetFaceVertices( 2, resT3.v[0], resT3.v[1], resT3.v[2] );
		resT3.v[ 3 ] = pointIndex;
		resT3.FixFaceOrientations( vertices );
		assert( srcT.SameOrientation( 2, resT3, 0, vertices ) );
		
		// res4
		srcT.GetFaceVertices( 3, resT4.v[0], resT4.v[1], resT4.v[2] );
		resT4.v[ 3 ] = pointIndex;
		resT4.FixFaceOrientations( vertices );
		assert( srcT.SameOrientation( 3, resT4, 0, vertices ) );

		// Adjust neighbors 

		resT1.neighbors[ 0 ] = srcT.neighbors[ 0 ];
		resT1.neighbors[ resT1.SharedFace( resT2, true ) ] = iResT2;
		resT1.neighbors[ resT1.SharedFace( resT3, true ) ] = iResT3;
		resT1.neighbors[ resT1.SharedFace( resT4, true ) ] = iResT4;

		resT2.neighbors[ 0 ] = srcT.neighbors[ 1 ];
		resT2.neighbors[ resT2.SharedFace( resT4, true ) ] = iResT4;
		resT2.neighbors[ resT2.SharedFace( resT3, true ) ] = iResT3;
		resT2.neighbors[ resT2.SharedFace( resT1, true ) ] = iResT1;

		resT3.neighbors[ 0 ] = srcT.neighbors[ 2 ];
		resT3.neighbors[ resT3.SharedFace( resT2, true ) ] = iResT2;
		resT3.neighbors[ resT3.SharedFace( resT4, true ) ] = iResT4;
		resT3.neighbors[ resT3.SharedFace( resT1, true ) ] = iResT1;

		resT4.neighbors[ 0 ] = srcT.neighbors[ 3 ];
		resT4.neighbors[ resT4.SharedFace( resT3, true ) ] = iResT3;
		resT4.neighbors[ resT4.SharedFace( resT2, true ) ] = iResT2;
		resT4.neighbors[ resT4.SharedFace( resT1, true ) ] = iResT1;
		
		for( int i = 0; i < 4; i++ ) {
			AdjustNeighborVicinity( resultingTetrahedra[ i ], 0, tetrahedra );
		}
#if _DEBUG
		for( int i = 0; i < 4; i++ ) {
			assert( tetrahedra[ resultingTetrahedra[ i ] ].CheckNeighbors( resultingTetrahedra[ i ], tetrahedra, vertices ) );
		}
#endif
	}

	/*
	================
	Delaunay3D::Flip23

	Takes two adjacent tetrahedra [abcd][bcde] sharing a face bcd, and splits them into 3 new
	tetrahedra linked by a single edge joining a-e
	================
	*/
	bool Delaunay3D::Flip23( const unsigned int tetrahedron1, const unsigned int tetrahedron2, 
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[3] ) {

		tetrahedron_t srcT1 = tetrahedra[ tetrahedron1 ];
		tetrahedron_t srcT2 = tetrahedra[ tetrahedron2 ];

		assert( srcT1.CheckFaceOrientations( vertices ) );
		assert( srcT2.CheckFaceOrientations( vertices ) );

		assert( srcT1.CheckNeighbors( tetrahedron1, tetrahedra, vertices ) );
		assert( srcT2.CheckNeighbors( tetrahedron2, tetrahedra, vertices ) );

		assert( srcT1.AdjacentTo( srcT2 ) );
				
		// The resulting 3 tetrahedra will all be adjacent to an edge linking the
		// non-shared vertices of the 2 source tetrahedra
		
		const int sharedFaceT1 = srcT1.SharedFace( srcT2, true ); // the shared face's index according to T1
		const int sharedFaceT2 = srcT2.SharedFace( srcT1, true ); // the shared face's index according to T2
																// note that the shared face is the same geometrically
																// but we're referring to it by its index in 
																// each tetrahedron's particular array

		const int a = srcT1.GetVertexOutsideFace( sharedFaceT1 );
		const int e = srcT2.GetVertexOutsideFace( sharedFaceT2 );
		
		int b,c,d;
		srcT1.GetFaceVertices( sharedFaceT1, b, c, d );

		// vertices A and E can be the same in a degenerate Case 1, where two adjacent tetrahedra also share
		// the vertex opposed to their shared face (that is, they're effectively the same tetrahedron with the 
		// faces declared in a different order). This can happen after inserting a vertex in the face of an existing
		// tetrahedron and causing N chained Case 4 till we close the cycle and reach the original point again.
		// This degenerate Case1 will generate 3 new (smaller) flat tetrahedra, that should eventually go away
		// with further flips
		// ! don't --> assert( a != e );
		assert( a != b && a != c && a != d );

		// assert outside vertex from T2 lies inside the sphere described by t1 (premise for a flip23)
		// the InSphere test requires points ABCD to be ordered such as D is in front of the plane defined
		// by ABC (clockwise order). Therefore since we've obtained our plane points from the tetrahedron
		// face, which points outwards, we'll need to invert them to make the 4th point be above the plane.

		assert( srcT1.IsFlat( vertices ) || InSphere(	vertices[ b ], vertices[ d ], vertices[ c ], // note inverted d,c
														vertices[ a ], // srcT1 4th vertex, above the inversed sharedFaceT1
														vertices[ e ]  // srcT2 4th vertex, which should be behind our tested plane
													 ) >= 0 
				); 

		// flip 23 is only possible if the edge a-e crosses the shared face
		// nevertheless on a Case 4, the segment a-e is going to be right upon one
		// of the edges from the face b,c,d, and we'll run into numeric issues.
		// Since we've already performed this check outside the function, we assume 
		// we know what we're doing here													 
		//REAL t,u,v;
		//if ( !SegmentTriangleIntersect_DoubleSided( vertices[ a ], vertices[ e ], vertices[ b ], vertices[ c ], vertices[ d ], t, u, v ) ) {
		//	return false;
		//}

		tetrahedra.preAllocate( tetrahedra.size() + 1 ); // ensure the list is not resized while we're holding references to its elements

		const unsigned int iResT1 = tetrahedron1; // reuse the source tetrahedron1 slot
		tetrahedron_t& resT1 = tetrahedra[ iResT1 ];
		resT1.Destroy( tetrahedra );
		const unsigned int iResT2 = tetrahedron2; // reuse the source tetrahedron2 slot
		tetrahedron_t& resT2 = tetrahedra[ iResT2 ];
		resT2.Destroy( tetrahedra );
		const unsigned int iResT3 = tetrahedra.size(); 
		tetrahedron_t& resT3 = tetrahedra.append();

		assert( !resT1.IsValid() );
		assert( !resT2.IsValid() );
		assert( !resT3.IsValid() );

		resultingTetrahedra[ 0 ] = iResT1;
		resultingTetrahedra[ 1 ] = iResT2;
		resultingTetrahedra[ 2 ] = iResT3;

		
		// Result 1

		resT1.v[ 0 ] = b;
		resT1.v[ 1 ] = d;
		resT1.v[ 2 ] = a;
		resT1.v[ 3 ] = e;
		resT1.FixFaceOrientations( vertices );

		{ // adjust boundary faces' neighborhood
			
			// get the external faces in the new tetrahedron...
			const int bFace1 = resT1.GetFaceFromVertices( a, b, d );
			const int bFace2 = resT1.GetFaceFromVertices( b, e, d );

			// ...and the corresponding faces in the source tetrahedra
			const int bFace1Src = srcT1.GetFaceFromVertices( a, b, d );
			const int bFace2Src = srcT2.GetFaceFromVertices( b, e, d );

			assert( bFace1 >= 0 && bFace2 >= 0 && bFace1Src >= 0 && bFace2Src >= 0 );
			assert( resT1.IsFlat( vertices ) || srcT1.SameOrientation( bFace1Src, resT1, bFace1, vertices ) );
			assert( resT1.IsFlat( vertices ) || srcT2.SameOrientation( bFace2Src, resT1, bFace2, vertices ) );

			// copy the neighbors
			resT1.neighbors[ bFace1 ] = srcT1.neighbors[ bFace1Src ];
			resT1.neighbors[ bFace2 ] = srcT2.neighbors[ bFace2Src ];

			// and adjust the neighborhood
			AdjustNeighborVicinity( iResT1, bFace1, tetrahedra );
			AdjustNeighborVicinity( iResT1, bFace2, tetrahedra );		
		}

		// Result 2

		resT2.v[ 0 ] = d;
		resT2.v[ 1 ] = c;
		resT2.v[ 2 ] = a;
		resT2.v[ 3 ] = e;
		resT2.FixFaceOrientations( vertices );


		{ // adjust boundary faces' neighborhood

			// get the external faces in the new tetrahedron...
			const int bFace1 = resT2.GetFaceFromVertices( c, a, d );
			const int bFace2 = resT2.GetFaceFromVertices( d, e, c );

			// ...and the corresponding faces in the source tetrahedra
			const int bFace1Src = srcT1.GetFaceFromVertices( c, a, d );
			const int bFace2Src = srcT2.GetFaceFromVertices( d, e, c );

			assert( bFace1 >= 0 && bFace2 >= 0 && bFace1Src >= 0 && bFace2Src >= 0 );
			assert( resT2.IsFlat( vertices ) || srcT1.SameOrientation( bFace1Src, resT2, bFace1, vertices ) );
			assert( resT2.IsFlat( vertices ) || srcT2.SameOrientation( bFace2Src, resT2, bFace2, vertices ) );

			// copy the neighbors
			resT2.neighbors[ bFace1 ] = srcT1.neighbors[ bFace1Src ];
			resT2.neighbors[ bFace2 ] = srcT2.neighbors[ bFace2Src ];

			// and adjust the neighborhood
			AdjustNeighborVicinity( iResT2, bFace1, tetrahedra );
			AdjustNeighborVicinity( iResT2, bFace2, tetrahedra );
		}

		// Result 3

		resT3.v[ 0 ] = c;
		resT3.v[ 1 ] = b;
		resT3.v[ 2 ] = a;
		resT3.v[ 3 ] = e;
		resT3.FixFaceOrientations( vertices );

		{ // adjust boundary faces' neighborhood

			// get the external faces in the new tetrahedron...
			const int bFace1 = resT3.GetFaceFromVertices( b, a, c );
			const int bFace2 = resT3.GetFaceFromVertices( c, e, b );

			// ...and the corresponding faces in the source tetrahedra
			const int bFace1Src = srcT1.GetFaceFromVertices( b, a, c );
			const int bFace2Src = srcT2.GetFaceFromVertices( c, e, b );

			assert( bFace1 >= 0 && bFace2 >= 0 && bFace1Src >= 0 && bFace2Src >= 0 );
			assert( resT3.IsFlat( vertices ) || srcT1.SameOrientation( bFace1Src, resT3, bFace1, vertices ) );
			assert( resT3.IsFlat( vertices ) || srcT2.SameOrientation( bFace2Src, resT3, bFace2, vertices ) );

			// copy the neighbors
			resT3.neighbors[ bFace1 ] = srcT1.neighbors[ bFace1Src ];
			resT3.neighbors[ bFace2 ] = srcT2.neighbors[ bFace2Src ];

			// and adjust the neighborhood
			AdjustNeighborVicinity( iResT3, bFace1, tetrahedra );
			AdjustNeighborVicinity( iResT3, bFace2, tetrahedra );
		}

		// adjust internal neighborhood
		resT1.neighbors[ resT1.SharedFace( resT2, true ) ] = iResT2;
		resT1.neighbors[ resT1.SharedFace( resT3, true ) ] = iResT3;
		resT2.neighbors[ resT2.SharedFace( resT1, true ) ] = iResT1;
		resT2.neighbors[ resT2.SharedFace( resT3, true ) ] = iResT3;
		resT3.neighbors[ resT3.SharedFace( resT1, true ) ] = iResT1;
		resT3.neighbors[ resT3.SharedFace( resT2, true ) ] = iResT2;

		
#if _DEBUG
		for( int i = 0; i < 3; i++ ) {
			assert( tetrahedra[ resultingTetrahedra[ i ] ].CheckNeighbors( resultingTetrahedra[ i ], tetrahedra, vertices ) );
		}
#endif
		return true;
	}

	int intCompare( const void* p0, const void* p1 );

	/*
	================
	Delaunay3D::Flip32

	Flips 3 adjacent tetrahedra into 2. The function reuses the two first slots in
	the tetrahedra array, but the later one given by tetrahedron3 must be marked as
	invalid.
	================
	*/
	void Delaunay3D::Flip32( const unsigned int tetrahedron1, const unsigned int tetrahedron2, const unsigned int tetrahedron3, 
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[2] ) {

		using namespace RenderLib::Math;

		tetrahedron_t srcT1 = tetrahedra[ tetrahedron1 ];
		tetrahedron_t srcT2 = tetrahedra[ tetrahedron2 ];
		tetrahedron_t srcT3 = tetrahedra[ tetrahedron3 ];

		assert( srcT1.CheckFaceOrientations( vertices ) );
		assert( srcT2.CheckFaceOrientations( vertices ) );
		assert( srcT3.CheckFaceOrientations( vertices ) );

		assert( srcT1.CheckNeighbors( tetrahedron1, tetrahedra, vertices ) );
		assert( srcT2.CheckNeighbors( tetrahedron2, tetrahedra, vertices ) );
		assert( srcT3.CheckNeighbors( tetrahedron3, tetrahedra, vertices ) );

		const unsigned int iResT1 = tetrahedron1; // reuse the source tetrahedron1 slot
		tetrahedron_t& resT1 = tetrahedra[ iResT1 ];
		resT1.Destroy( tetrahedra );
		const unsigned int iResT2 = tetrahedron2; // reuse the source tetrahedron2 slot
		tetrahedron_t& resT2 = tetrahedra[ iResT2 ];
		resT2.Destroy( tetrahedra );

		assert( !resT1.IsValid() );
		assert( !resT2.IsValid() );

		resultingTetrahedra[ 0 ] = iResT1;
		resultingTetrahedra[ 1 ] = iResT2;

		// Source tetrahedra: [abdp][bcdp][cadp]
		// resulting tetrahedra: [abcd][abcp]

		int a = -1, b = -1, c = -1, d = -1, p = -1;

		{		
			const int sharedFace12 = srcT1.SharedFace( srcT2, true );
			const int sharedFace13 = srcT1.SharedFace( srcT3, true );
			const int sharedFace23 = srcT2.SharedFace( srcT3, true );
			assert( sharedFace12 >= 0 && sharedFace13 >= 0 && sharedFace23 >= 0 );

			int sharedFaceVertices[ 9 ];
			srcT1.GetFaceVertices( sharedFace12, sharedFaceVertices[ 3 * 0 + 0 ], sharedFaceVertices[ 3 * 0 + 1 ], sharedFaceVertices[ 3 * 0 + 2 ] );
			srcT1.GetFaceVertices( sharedFace13, sharedFaceVertices[ 3 * 1 + 0 ], sharedFaceVertices[ 3 * 1 + 1 ], sharedFaceVertices[ 3 * 1 + 2 ] );
			srcT2.GetFaceVertices( sharedFace23, sharedFaceVertices[ 3 * 2 + 0 ], sharedFaceVertices[ 3 * 2 + 1 ], sharedFaceVertices[ 3 * 2 + 2 ] );

			// These shared faces should all have 2 vertices in common (d and p) and 
			// the remaining vertex only shared with 1 other face (a, b, c)
			qsort( sharedFaceVertices, 9, sizeof( int ), intCompare );

			int uniqueVertices[5];
			int occurrence[5];
			memset( occurrence, 0, 5 * sizeof(int) );
			int uv = 1;
			uniqueVertices[ 0 ] = sharedFaceVertices[ 0 ];
			occurrence[ 0 ] = 1;
			for( int i = 1; i < 9; i++ ) {
				bool found = false;
				for( int j = 0; j < uv; j++ ) {
					if( uniqueVertices[ j ] == sharedFaceVertices[ i ] ) {
						occurrence[ j ]++;
						found = true;
						break;
					}
				}
				if ( !found ) {
					uniqueVertices[ uv ] = sharedFaceVertices[ i ];
					occurrence[ uv ] = 1;
					uv ++;
				}
			}

			assert( uv == 5 );

			int dp[2];
			int abc[3];
			int iDp = 0;
			int iAbc = 0;
			for( int i = 0; i < 5; i++ ) {
				if( occurrence[ i ] == 3 ) {
					dp[ iDp++ ] = uniqueVertices[ i ];
					assert( iDp <= 2 );
				} else if( occurrence[ i ] == 1 ) {
					abc[ iAbc++ ] = uniqueVertices[ i ]; 
					assert( iAbc <= 3 );
				} else {
					assert( false );
				}
			}

			a = abc[ 0 ];
			b = abc[ 1 ];
			c = abc[ 2 ];
			const Vector3<REAL> facePlaneNormal = Vector3<REAL>::normalize( Vector3<REAL>::cross( vertices[ c ] - vertices[ a ], vertices[ b ] - vertices[ a ] ) );
			const REAL facePlaneDist = Vector3<REAL>::dot(facePlaneNormal, vertices[ a ].fromOrigin());
			if ( Vector3<REAL>::dot(facePlaneNormal, vertices[ dp[ 0 ] ].fromOrigin()) - facePlaneDist > -1e-4 ) { // sort it so p falls below ABC and d above
				d = dp[ 0 ];
				p = dp[ 1 ];
			} else {
				d = dp[ 1 ];
				p = dp[ 0 ];
			}
			assert( Vector3<REAL>::dot(facePlaneNormal, vertices[ d ].fromOrigin()) - facePlaneDist > -1e-4 );
			assert( Vector3<REAL>::dot(facePlaneNormal, vertices[ p ].fromOrigin()) - facePlaneDist < 1e-4 );
		}
		
		assert( a >= 0 && b >= 0 && c >= 0 && d >= 0 && p >= 0 );

		// destroy the unused tetrahedron
		tetrahedra[ tetrahedron3 ].Destroy( tetrahedra );

		// Result 1

		resT1.v[ 0 ] = a;
		resT1.v[ 1 ] = c;
		resT1.v[ 2 ] = b;
		resT1.v[ 3 ] = d;
		resT1.FixFaceOrientations( vertices );

		{ // sort out neighbors
			const int sf1 = srcT1.SharedFace( resT1, false );
			const int sf2 = srcT2.SharedFace( resT1, false );
			const int sf3 = srcT3.SharedFace( resT1, false );
			const int sf4 = resT1.GetFaceFromVertices( a, c, b );
			assert( sf1 >= 0 && sf2 >= 0 && sf3 >= 0 && sf4 >= 0 );

			int v[3];

			srcT1.GetFaceVertices( sf1, v[0], v[1], v[2] );
			const int r1sf = resT1.GetFaceFromVertices( v[0], v[1], v[2] );
			resT1.neighbors[ r1sf ] = srcT1.neighbors[ sf1 ];

			srcT2.GetFaceVertices( sf2, v[0], v[1], v[2] );
			const int r2sf = resT1.GetFaceFromVertices( v[0], v[1], v[2] );
			resT1.neighbors[ r2sf ] = srcT2.neighbors[ sf2 ];

			srcT3.GetFaceVertices( sf3, v[0], v[1], v[2] );
			const int r3sf = resT1.GetFaceFromVertices( v[0], v[1], v[2] );
			resT1.neighbors[ r3sf ] = srcT3.neighbors[ sf3 ];

			AdjustNeighborVicinity( iResT1, r1sf, tetrahedra );
			AdjustNeighborVicinity( iResT1, r2sf, tetrahedra );
			AdjustNeighborVicinity( iResT1, r3sf, tetrahedra );

			resT1.neighbors[ sf4 ] = iResT2;
		}		

		// Result 2

		resT2.v[ 0 ] = a;
		resT2.v[ 1 ] = b;
		resT2.v[ 2 ] = c;
		resT2.v[ 3 ] = p;
		resT2.FixFaceOrientations( vertices );

		{ // sort out neighbors
			const int sf1 = srcT1.SharedFace( resT2, false );
			const int sf2 = srcT2.SharedFace( resT2, false );
			const int sf3 = srcT3.SharedFace( resT2, false );
			const int sf4 = resT2.GetFaceFromVertices( a, b, c );
			assert( sf1 >= 0 && sf2 >= 0 && sf3 >= 0 && sf4 >= 0 );

			int v[3];

			srcT1.GetFaceVertices( sf1, v[0], v[1], v[2] );
			const int r1sf = resT2.GetFaceFromVertices( v[0], v[1], v[2] );
			resT2.neighbors[ r1sf ] = srcT1.neighbors[ sf1 ];

			srcT2.GetFaceVertices( sf2, v[0], v[1], v[2] );
			const int r2sf = resT2.GetFaceFromVertices( v[0], v[1], v[2] );
			resT2.neighbors[ r2sf ] = srcT2.neighbors[ sf2 ];

			srcT3.GetFaceVertices( sf3, v[0], v[1], v[2] );
			const int r3sf = resT2.GetFaceFromVertices( v[0], v[1], v[2] );
			resT2.neighbors[ r3sf ] = srcT3.neighbors[ sf3 ];

			AdjustNeighborVicinity( iResT2, r1sf, tetrahedra );
			AdjustNeighborVicinity( iResT2, r2sf, tetrahedra );
			AdjustNeighborVicinity( iResT2, r3sf, tetrahedra );

			resT2.neighbors[ sf4 ] = iResT1;
		}		
		
#if _DEBUG
		for( int i = 0; i < 2; i++ ) {
			assert( tetrahedra[ resultingTetrahedra[ i ] ].CheckNeighbors( resultingTetrahedra[ i ], tetrahedra, vertices ) );
		}
#endif
	}

	/*
	================
	Delaunay3D::Flip44

	Flips 4 adjacent tetrahedra [abcd][acde][bcdf][cdef] sharing a common edge c-d
	into 4 adjacent tetrahedra [abce][abed][bcef][bdef] sharing a common edge b-e
	================
	*/
	void Delaunay3D::Flip44( const unsigned int tetrahedron1, const unsigned int tetrahedron2, const unsigned int tetrahedron3, const unsigned int tetrahedron4,
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[4] ) {

		
		/* The order of the input tetrahedra is:

					 a                   a
					/|\                 /|\
				   / | \               / | \
				  /  |  \             /  |  \
				 / 1 | 3 \           / 1 | 3 \
			   b/____|d___\e  -->  c/____|e___\d
				\   c|    /         \   b|    /
				 \ 2 | 4 /           \ 2 | 4 /
				  \  |  /             \  |  /
				   \ | /               \ | /
					\|/                 \|/
					 f                   f
		*/

		tetrahedron_t srcT1 = tetrahedra[ tetrahedron1 ];
		tetrahedron_t srcT2 = tetrahedra[ tetrahedron2 ];
		tetrahedron_t srcT3 = tetrahedra[ tetrahedron3 ];
		tetrahedron_t srcT4 = tetrahedra[ tetrahedron4 ];

		assert( srcT1.AdjacentTo( srcT2 ) );
		assert( srcT1.AdjacentTo( srcT3 ) );
		assert( srcT2.AdjacentTo( srcT4 ) );
		assert( srcT3.AdjacentTo( srcT4 ) );

		assert( srcT1.CheckFaceOrientations( vertices ) );
		assert( srcT2.CheckFaceOrientations( vertices ) );
		assert( srcT3.CheckFaceOrientations( vertices ) );
		assert( srcT4.CheckFaceOrientations( vertices ) );

		assert( srcT1.CheckNeighbors( tetrahedron1, tetrahedra, vertices ) );
		assert( srcT2.CheckNeighbors( tetrahedron2, tetrahedra, vertices ) );
		assert( srcT3.CheckNeighbors( tetrahedron3, tetrahedra, vertices ) );
		assert( srcT4.CheckNeighbors( tetrahedron4, tetrahedra, vertices ) );

		const int iResT1 = tetrahedron1; // reuse the source tetrahedron1 slot
		tetrahedron_t& resT1 = tetrahedra[ iResT1 ];
		resT1.Destroy( tetrahedra );
		const int iResT2 = tetrahedron2; // reuse the source tetrahedron2 slot
		tetrahedron_t& resT2 = tetrahedra[ iResT2 ];
		resT2.Destroy( tetrahedra );
		const int iResT3 = tetrahedron3; // reuse the source tetrahedron3 slot
		tetrahedron_t& resT3 = tetrahedra[ iResT3 ];
		resT3.Destroy( tetrahedra );
		const int iResT4 = tetrahedron4; // reuse the source tetrahedron4 slot
		tetrahedron_t& resT4 = tetrahedra[ iResT4 ];
		resT4.Destroy( tetrahedra );

		assert( !resT1.IsValid() );
		assert( !resT2.IsValid() );
		assert( !resT3.IsValid() );
		assert( !resT4.IsValid() );

		resultingTetrahedra[ 0 ] = iResT1;
		resultingTetrahedra[ 1 ] = iResT2;
		resultingTetrahedra[ 2 ] = iResT3;
		resultingTetrahedra[ 3 ] = iResT4;
			
		// extract the vertices by finding the shared faces
		// between each pair of tetrahedra and retrieving
		// the outlying vertices on each side. This way we
		// avoid dealing with the order in which the shared
		// face vertices are declared

		int a, b, c, d, e, f;
		int sf;

		sf = srcT1.SharedFace( srcT2, true ); // [bcd]
		assert( sf >= 0 );
		a = srcT1.GetVertexOutsideFace( sf );
		
		sf = srcT2.SharedFace( srcT1, true ); // [bcd]
		assert( sf >= 0 );
		f = srcT2.GetVertexOutsideFace( sf );

		sf = srcT1.SharedFace( srcT3, true ); // [acd]
		assert( sf >= 0 );
		b = srcT1.GetVertexOutsideFace( sf );

		sf = srcT3.SharedFace( srcT1, true ); // [acd]
		assert( sf >= 0 );
		e = srcT3.GetVertexOutsideFace( sf );

		// The only remaining vertices are c and d
		// Since we know they belong to each one of
		// the source tetrahedra, just identify the 
		// not-yet assigned vertices on any of the
		// source tetrahedra

		// srcT1 [abcd] --> we have a, b already
		c = -1; d = -1;
		for( int i = 0; i < 4 && c < 0 && d < 0; i++ ) {
			if ( srcT1.GetVertexOutsideFace( i ) == b ) {
				int v[3];
				srcT1.GetFaceVertices( i, v[0], v[1], v[2] );
				for( int j = 0; j < 3; j++ ) {
					if ( a == v[ j ] ) {
						c = v[ ( j + 1 ) % 3 ];
						d = v[ ( j + 2 ) % 3 ];
						break;
					}
				}
			}
		} 
		assert( c >= 0 && d >= 0 );

		memcpy( resT1.v, srcT1.v, 4 * sizeof(int) );
		memcpy( resT2.v, srcT2.v, 4 * sizeof(int) );
		memcpy( resT3.v, srcT3.v, 4 * sizeof(int) );
		memcpy( resT4.v, srcT4.v, 4 * sizeof(int) );
		for( int i = 0; i < 4; i++ ) {
			if ( resT1.v[ i ] == d ) resT1.v[ i ] = e;
			if ( resT2.v[ i ] == d ) resT2.v[ i ] = e;
			if ( resT3.v[ i ] == c ) resT3.v[ i ] = b;
			if ( resT4.v[ i ] == c ) resT4.v[ i ] = b;
		}

		//resT1.v[0] = a;
		//resT1.v[1] = b;
		//resT1.v[2] = e;
		//resT1.v[3] = d;
		//resT1.FixFaceOrientations( vertices );

		//resT2.v[0] = f;
		//resT2.v[1] = d;
		//resT2.v[2] = e;
		//resT2.v[3] = b;
		//resT2.FixFaceOrientations( vertices );

		//resT3.v[0] = a;
		//resT3.v[1] = c;
		//resT3.v[2] = b;
		//resT3.v[3] = e;
		//resT3.FixFaceOrientations( vertices );

		//resT4.v[0] = b;
		//resT4.v[1] = c;
		//resT4.v[2] = e;
		//resT4.v[3] = f;
		//resT4.FixFaceOrientations( vertices );

		// Result 1
		
		{ // sort out neighbors

			// get the external faces in the new tetrahedron...
			const int sf1 = srcT1.GetFaceFromVertices( a, b, c );
			const int sf3 = srcT3.GetFaceFromVertices( a, c, e );
			const int rf1 = resT1.GetFaceFromVertices( a, b, c );
			const int rf3 = resT1.GetFaceFromVertices( a, c, e );
			assert( sf1 >= 0 && sf3 >= 0 && rf1 >= 0 && rf3 >= 0 );

			resT1.neighbors[ rf1 ] = srcT1.neighbors[ sf1 ];
			resT1.neighbors[ rf3 ] = srcT3.neighbors[ sf3 ];

			AdjustNeighborVicinity( iResT1, rf1, tetrahedra );
			AdjustNeighborVicinity( iResT1, rf3, tetrahedra );
		}

		// Result 2

		{ // sort out neighbors
			const int sf2 = srcT2.GetFaceFromVertices( b, c, f );
			const int sf4 = srcT4.GetFaceFromVertices( c, e, f );
			const int rf2 = resT2.GetFaceFromVertices( b, c, f );
			const int rf4 = resT2.GetFaceFromVertices( c, e, f );
			assert( sf2 >= 0 && sf4 >= 0 && rf2 >= 0 && rf4 >= 0 );

			resT2.neighbors[ rf2 ] = srcT2.neighbors[ sf2 ];
			resT2.neighbors[ rf4 ] = srcT4.neighbors[ sf4 ];

			AdjustNeighborVicinity( iResT2, rf2, tetrahedra );
			AdjustNeighborVicinity( iResT2, rf4, tetrahedra );
		}

		// Result 3


		{ // sort out neighbors
			const int sf1 = srcT1.GetFaceFromVertices( a, b, d );
			const int sf3 = srcT3.GetFaceFromVertices( a, d, e );
			const int rf1 = resT3.GetFaceFromVertices( a, b, d );
			const int rf3 = resT3.GetFaceFromVertices( a, d, e );
			assert( sf1 >= 0 && sf3 >= 0 && rf1 >= 0 && rf3 >= 0 );

			resT3.neighbors[ rf1 ] = srcT1.neighbors[ sf1 ];
			resT3.neighbors[ rf3 ] = srcT3.neighbors[ sf3 ];

			AdjustNeighborVicinity( iResT3, rf1, tetrahedra );
			AdjustNeighborVicinity( iResT3, rf3, tetrahedra );
		}

		// Result 4


		{ // sort out neighbors
			
			const int sf2 = srcT2.GetFaceFromVertices( b, d, f );
			const int sf4 = srcT4.GetFaceFromVertices( d, e, f );
			const int rf2 = resT4.GetFaceFromVertices( b, d, f );
			const int rf4 = resT4.GetFaceFromVertices( d, e, f );

			assert( sf2 >= 0 && sf4 >= 0 && rf2 >= 0 && rf4 >= 0 );

			resT4.neighbors[ rf2 ] = srcT2.neighbors[ sf2 ];
			resT4.neighbors[ rf4 ] = srcT4.neighbors[ sf4 ];

			AdjustNeighborVicinity( iResT4, rf2, tetrahedra );
			AdjustNeighborVicinity( iResT4, rf4, tetrahedra );
		}

		{
			// Fix adjacency between resulting tetrahedra
			resT1.neighbors[ resT1.GetFaceFromVertices( b, c, e ) ] = iResT2;
			resT1.neighbors[ resT1.GetFaceFromVertices( a, b, e ) ] = iResT3;
			resT2.neighbors[ resT2.GetFaceFromVertices( b, c, e ) ] = iResT1;
			resT2.neighbors[ resT2.GetFaceFromVertices( b, e, f ) ] = iResT4;
			resT3.neighbors[ resT3.GetFaceFromVertices( a, b, e ) ] = iResT1;
			resT3.neighbors[ resT3.GetFaceFromVertices( b, d, e ) ] = iResT4;
			resT4.neighbors[ resT4.GetFaceFromVertices( b, d, e ) ] = iResT3;
			resT4.neighbors[ resT4.GetFaceFromVertices( b, e, f ) ] = iResT2;
		}

#if _DEBUG
		for( int i = 0; i < 4; i++ ) {
			assert( tetrahedra[ resultingTetrahedra[ i ] ].CheckNeighbors( resultingTetrahedra[ i ], tetrahedra, vertices ) );	
		}
#endif
		resT1.FixFaceOrientations( vertices );
		resT2.FixFaceOrientations( vertices );
		resT3.FixFaceOrientations( vertices );
		resT4.FixFaceOrientations( vertices );
	}

	//////////////////////////////////////////////////////////////////////////
	// Auxiliary functions
	//////////////////////////////////////////////////////////////////////////

	/*
	================
	Delaunay3D::Containingtetrahedron

	Generates a big tetrahedron t containing the bounding sphere
	The vertices of t do *not* belong to the provided point set,
	and therefore are returned as < 0 indices referencing the
	temp vertices array obtained by ::TempVertices();
	================
	*/
	void Delaunay3D::Containingtetrahedron( const RenderLib::Math::Point3f& center, const float radius, // circumsphere
											tetrahedron_t& t,
											CoreLib::List< Point >& pointSet ) {

		// from the Wikipedia: 
		// For a regular tetrahedron of edge length L: radius = L / sqrt( 24 )
		// http://en.wikipedia.org/wiki/Tetrahedron#Formulas_for_regular_tetrahedron

		const REAL sqRoot = sqrt( 24.0 );
		const REAL L = radius * sqRoot;

		/*
					L/2       L/2
				O---------+----------      regular triangle, A = 60 deg
				\ A       |       A /      h = h1 + h2 = L * sin(A)
				 \        |h1      /       angle (O+),(OP) = A/2 = 30 deg
				  \       P       /        tan(A/2) = h1 / (L/2)
				   \      |      /         h1 = tan(30) * L / 2
				 L  \     |     / L		   h2 = h - h1 = L * sin(60) - h1
					 \    |h2  /
					  \   |   /
					   \  |  /
						\ A /
						 \|/

		*/

		const REAL h1 = tan( DEG2RAD(30.0) ) * L / 2;
		const REAL h2 = sin( DEG2RAD(60.0) ) *  L - h1 ;

		/*
			y
			|
			|
			+----> x
		   /
		  z		p1 ------------_- p2
				  \     ___--- /        p1 = ( P.x - L/2,	P.y + radius,		P.z - h1 )
				   \_---      /         p2 = ( P.x + L/2,	P.y + radius,		P.z - h1 )
				  p3\    P   /          p3 = ( P.x,			P.y + radius,		P.z + h2 )
					 \      /           p4 = ( P.x,			P.y + radius - h,	P.z )
					  \    /
					   \  /
						p4
		*/ 

		const REAL margin = L * (REAL)0.005;

		const Point p1( (float)( center.x - L / 2 - margin ), (float)( center.y + radius + margin ), (float)( center.z - h1 - margin ) );
		const Point p2( (float)( center.x + L / 2 + margin ), (float)( center.y + radius + margin ), (float)( center.z - h1 - margin ) );
		const Point p3( (float)( center.x ), (float)( center.y + radius + margin ), (float)( center.z + h2 + margin ) );
		const Point p4( (float)( center.x ), (float)( center.y + radius - h1 - h2 + margin ), (float)( center.z ) );

		t.v[ 0 ] = pointSet.append( p1 );
		t.v[ 1 ] = pointSet.append( p2 );
		t.v[ 2 ] = pointSet.append( p3 );
		t.v[ 3 ] = pointSet.append( p4 );
	}
	/*
	================
	Delaunay3D::AdjustNeighborVicinity

	Given a tetrahedron and a face, ensures the potential adjacent tetrahedron sharing
	that face points to the given tetrahedron.
	================
	*/
	void Delaunay3D::AdjustNeighborVicinity( const int iT, const int f, CoreLib::List< tetrahedron_t >& tetrahedra ) {
		const tetrahedron_t& T = tetrahedra[ iT ];
		assert( T.IsValid() && "T.IsValid()" );
		if( T.neighbors[ f ] >= 0 ) { // adjust our neighbor's vicinity
			tetrahedron_t& n = tetrahedra[ T.neighbors[ f ] ];
			if ( n.IsValid() ) {
				int v[3];
				T.GetFaceVertices( f, v[0], v[1], v[2] );
				const int sharedFace = n.GetFaceFromVertices( v[0], v[2], v[1] ); // reverse face
				assert( sharedFace >= 0 && "sharedFace >= 0" );
				n.neighbors[ sharedFace ] = iT;
			}
		}
	}

	/*
	================
	Delaunay3D::Walk

	Returns the index of the tetrahedron containing p, or -1 if not found. 
	It retrieves the result by searching from a given sourceT tetrahedron index. 
	================
	*/
	int Delaunay3D::Walk( const Point& p, const int sourceT, 
						  const CoreLib::List< Point >& vertices,
						  const CoreLib::List< tetrahedron_t >& tetrahedra ) {
		if ( sourceT < 0 || (size_t)sourceT >= tetrahedra.size() ) {
			assert( false );
			return -1;
		}

		CoreLib::List< bool > visited;
		visited.resize( tetrahedra.size(), true );
		for( size_t i = 0; i < tetrahedra.size(); i++ ) {
			visited[ i ] = false;
		}
		CoreLib::List< int > traversal( 64 );
		
		int t = sourceT;

		while( true ) {
			
			const tetrahedron_t& tetrahedron = tetrahedra[ t ];
			visited[ t ] = true;
			traversal.append( t );
			bool step = false;

			if ( tetrahedron.IsValid() ) {

				if ( Inside( p, tetrahedron, vertices ) ) {
					return t;
				}

				// pick adjacent neighbor such as p lies on the positive side
				// of their shared face
				int a, b, c;
				for( int i = 0; i < 4; i++ ) {
					if ( tetrahedron.neighbors[ i ] >= 0 ) {
						if ( visited[ tetrahedron.neighbors[ i ] ] ) {
							continue;
						}
						tetrahedron.GetFaceVertices( i, a, b, c );
						if ( Orient( vertices[ a ], vertices[ b ], vertices[ c ], p ) > 0 ) {
							t = tetrahedron.neighbors[ i ];
							step = true;
							break;
						}
					}
				}
			}

			if ( !step ) {
				// we reached a dead end. Retry from another source tetrahedron

				bool found = false;
				for( size_t i = 0; i < visited.size(); i++ ) {
					if ( !visited[ i ] ) {
						t = i;
						found = true;
						break;
					}
				}

				traversal.resize( 0, false );

				if ( !found ) {
					assert( false );
					return -1;
				}
			}			
		}
		
		assert( false );
		return -1;
	}

	/*
	================
	Delaunay3D::Flip

	Flips two given tetrahedra: T and Ta, which are non-Delaunay, 
	into a Delaunay configuration using bistellar flips. The
	resulting tetrahedra's adjacents will need to be tested for
	Delaunay-ness, and they're inserted into the needTesting stack
	================
	*/
	
	void Delaunay3D::Flip( const int iT, const int iTa, const int p,
						   const CoreLib::List< Point >& vertices, 
						   CoreLib::List< tetrahedron_t >& tetrahedra,
						   std::stack< unsigned int >& needTesting ) {
		
		using namespace RenderLib::Geometry;

		tetrahedron_t& T = tetrahedra[ iT ];
		tetrahedron_t& Ta = tetrahedra[ iTa ];

		assert( T.AdjacentTo( Ta ) );
		assert( T.ContainsVertex( p ) );

		const int TSharedFace = T.SharedFace( Ta, true );
		assert( TSharedFace >= 0 );

		int sharedVertices[3];
		T.GetFaceVertices( TSharedFace, sharedVertices[ 0 ], sharedVertices[ 1 ], sharedVertices[ 2 ] );

		const int d = Ta.GetVertexOutsideFace( Ta.SharedFace( T, true ) );
		assert( d >= 0 );

		int Case = -1;

		const Point& pA = vertices[ sharedVertices[ 0 ] ];
		const Point& pB = vertices[ sharedVertices[ 1 ] ];
		const Point& pC = vertices[ sharedVertices[ 2 ] ];
		const Point& pP = vertices[ p ];
		const Point& pD = vertices[ d ];

		if( Coplanar( pA, pB, pC, pP ) ) {
			// case 4 = a, b, c, p are coplanar. T is flat.
			Case = 4;
			assert( T.IsFlat( vertices ) );
		} else if ( Coplanar( pA, pB, pD, pP ) ) {
			Case = 31;
		} else if ( Coplanar( pA, pC, pD, pP ) ) {
			Case = 32;
		} else if ( Coplanar( pB, pC, pD, pP ) ) {
			Case = 33;
		} else {
			REAL t,v,w;
			if ( SegmentTriangleIntersect_DoubleSided( pP, pD, pA, pB, pC, t, v, w ) ) {
				// the segment p-d crosses the shared face. Therefore the union between T and Ta
				// is convex
				Case = 1;
			} else {
				// the segment p-d does not cross the shared face
				Case = 2;
			}
		}

#if _DEBUG
		if ( Case != 4 ) {
			assert( !T.IsFlat( vertices ) );
		}
#endif

		switch( Case ) {
			case 1:
				{
					// Case 1: only one face of Ta is visible from the non-shared 
					// vertex of T, and therefore the union of  and a is a convex 
					// polyhedron. In this case, a flip23 is performed.

					unsigned int result[3];
					if ( Flip23( iT, iTa, tetrahedra, vertices, result ) ) {
						for( int i = 0; i < 3; i++ ) {
							needTesting.push( result[ i ] );
						} 
					} 
				}
				break;
			case 4:
				{			
					// According to the paper:

					// Case 4: Degenerate case: the tetrahedron abcp is flat
					// the vertex p is directly on one edge of the
					// face abc (assume it is ab here). Thus, p lies in the
					// same plane as abc, but also as abd. The tetrahedron
					// abcp is obviously flat. The only reason for
					// such a case is because p was inserted directly on
					// one edge of T in the first place (the edge ab), and
					// the first flip14 to split the tetrahedron containing
					// p created two flat tetrahedra. Because p was
					// inserted on the edge ab, all the tetrahedra in T
					// incident to ab must be split into two tetrahedra.
					// This could be done with the degenerate flip12, but
					// it can also be done simply with a sequence of flip23
					// and flip32. When the case #4 happens, it suffices
					// to perform a flip23 on T and Ta. Obviously, another
					// flat tetrahedron abdp will be created, but
					// this one will deleted by another flip.

					// However we can't apply a flip23 blindly! Since this is an edge case
					// unlike when we're in Case 1, we can't be sure there won't be yet
					// another (flat) tetrahedron sharing the faces of the tetrahedra we're
					// intending to swap (in this case one of the 3 resulting tetrahedra
					// would completely overlap the external one, and this is invalid). 
					// The idea is performing a fli23 -> flip32 sequence according to the paper
					// so we'll first check for the existence of a tetrahedron linking our
					// 2 candidates, which would give us the config for the eventual flip32 directly
					// such as the Case #2:

					// In short:
					//		Got two tetrahedra T and Ta in Case 4 (T is flat)
					//			* if there exists Tb adjacent to both T and Ta --> perform flip32 (like Case 2)
					//			* otherwise --> perform flip23 (like Case 1) resulting in 
					//			another flat tetrahedron that we'll remove later on

					bool fixed = false;
					for( int s = 0; !fixed && s < 3; s++ ) {
						const int a = sharedVertices[ s ];
						const int b = sharedVertices[ ( s + 1 ) % 3 ];
						const int abp = T.GetFaceFromVertices( a, b, p );
						const int bap = Ta.GetFaceFromVertices( b, a, d );
#if _DEBUG
						if ( abp < 0 ) {
							assert( T.GetFaceFromVertices(b, a, p ) < 0 );
							assert( T.GetFaceFromVertices(b, p, a ) < 0 );
						}
						if ( bap < 0 ) {
							assert( Ta.GetFaceFromVertices( a, b, p ) < 0 );
							assert( Ta.GetFaceFromVertices( a, p, b ) < 0 );
						}
#endif
						if ( abp >= 0 && bap >= 0 ) {
							const int iTb1 = T.neighbors[ abp ];
							const int iTb2 = Ta.neighbors[ bap ];
							if ( iTb1 >= 0 && iTb1 == iTb2 ) {
#if _DEBUG
								tetrahedron_t& Tb = tetrahedra[ iTb1 ];								
								assert( Tb.GetVertexOutsideFace( Tb.GetFaceFromVertices( a, b, p ) ) == d );
#endif
								fixed = true;
								unsigned int result[2];
								Flip32( iT, iTa, iTb1, tetrahedra, vertices, result );
								needTesting.push( result[ 0 ] );
								needTesting.push( result[ 1 ] );
							}
						}
					}

					if ( !fixed ) {
						unsigned int result[3];
						if ( Flip23( iT, iTa, tetrahedra, vertices, result ) ) {
							for( int i = 0; i < 3; i++ ) {
								needTesting.push( result[ i ] );
							} 
						} else {
							// flip not possible
							if ( Case == 4 ) {
								// T was flat and it was not possible to get rid of it. Erase. 
								//T.Destroy( tetrahedra );
							}
						} 
					}
				}
				break;
			case 2:
				{
					// Case 2: two faces of Ta are visible, and therefore
					// the union of T and Ta is non-convex. If there exists
					// a tetrahedron Tb = abpd such that the
					// edge ab is shared by  T, Ta and Tb, then a flip32 can
					// be performed. If there exists no such tetrahedron,
					// then no flip is performed. The non-locally Delaunay
					// facet abc will be ‘rectified’ by another flip
					// performed on adjacent tetrahedra.

					bool fixed = false;
					for( int s = 0; !fixed && s < 3; s++ ) {
						const int a = sharedVertices[ s ];
						const int b = sharedVertices[ ( s + 1 ) % 3 ];
						const int abp = T.GetFaceFromVertices( a, b, p );
						const int bap = Ta.GetFaceFromVertices( b, a, d );
#if _DEBUG
						if ( abp < 0 ) {
							assert( T.GetFaceFromVertices(b, a, p ) < 0 );
							assert( T.GetFaceFromVertices(b, p, a ) < 0 );
						}
						if ( bap < 0 ) {
							assert( Ta.GetFaceFromVertices( a, b, p ) < 0 );
							assert( Ta.GetFaceFromVertices( a, p, b ) < 0 );
						}
#endif
						if ( abp >= 0 && bap >= 0 ) {
							const int iTb1 = T.neighbors[ abp ];
							const int iTb2 = Ta.neighbors[ bap ];
							if ( iTb1 >= 0 && iTb1 == iTb2 ) {
#if _DEBUG
								tetrahedron_t& Tb = tetrahedra[ iTb1 ];								
								assert( Tb.GetVertexOutsideFace( Tb.GetFaceFromVertices( a, b, p ) ) == d );
#endif
								fixed = true;
								unsigned int result[2];
								Flip32( iT, iTa, iTb1, tetrahedra, vertices, result );
								needTesting.push( result[ 0 ] );
								needTesting.push( result[ 1 ] );
								assert( tetrahedra[ result[ 0 ] ].ContainsVertex( a ) || tetrahedra[ result[ 1 ] ].ContainsVertex( a ) );
								assert( tetrahedra[ result[ 0 ] ].ContainsVertex( b ) || tetrahedra[ result[ 1 ] ].ContainsVertex( b ) );
								assert( tetrahedra[ result[ 0 ] ].ContainsVertex( p ) );
								assert( tetrahedra[ result[ 1 ] ].ContainsVertex( p ) );
								assert( tetrahedra[ result[ 0 ] ].ContainsVertex( d ) );
								assert( tetrahedra[ result[ 1 ] ].ContainsVertex( d ) );
#if _DEBUG
								const int c = sharedVertices[ ( s + 2 ) % 3 ];
								assert( tetrahedra[ result[ 0 ] ].ContainsVertex( c ) );
								assert( tetrahedra[ result[ 1 ] ].ContainsVertex( c ) );
#endif

								assert( !tetrahedra[ iTb1 ].IsValid() );
							}
						}
					}
				}
				break;

			case 31:
			case 32:
			case 33:
				{
					// Degenerate case: p,a,b,d are coplanar

					// the line segment pd intersects directly one
					// edge of abc (assume it is ab here). Thus, vertices p,
					// d, a and b are coplanar. Observe that a flip23, that
					// would create a flat tetrahedron abdp, is possible if
					// and only if T and Ta are in config44
					// with two other tetrahedra Tb and Tc, such that the
					// edge ab is incident to T, Ta, Tb and Tc. Since the
					// four tetrahedra are in config44, a flip44 can be
					// performed. If not, no flip is performed.

					int sharedSegmentA, sharedSegmentB;
					if ( Case == 31 ) {
						// ab
						sharedSegmentA = sharedVertices[ 0 ];
						sharedSegmentB = sharedVertices[ 1 ];
					} else if ( Case == 32 ) {
						// ac
						sharedSegmentA = sharedVertices[ 0 ];
						sharedSegmentB = sharedVertices[ 2 ];
					} else {
						// bc
						sharedSegmentA = sharedVertices[ 1 ];
						sharedSegmentB = sharedVertices[ 2 ];
					}
					assert( Coplanar( vertices[ sharedSegmentA ], vertices[ sharedSegmentB ], vertices[ p ], vertices[ d ] ) );
					int faceT = T.GetFaceFromVertices( sharedSegmentA, sharedSegmentB, p );				
					int faceTa = Ta.GetFaceFromVertices( sharedSegmentB, sharedSegmentA, d );
					assert( faceT >= 0 && faceTa >= 0 );

					int iNeighborT = T.neighbors[ faceT ];
					int iNeighborTa = Ta.neighbors[ faceTa ];
					if ( iNeighborT >= 0 && iNeighborTa >= 0 ) {
						tetrahedron_t& Tb = tetrahedra[ iNeighborT ];
						tetrahedron_t& Tc = tetrahedra[ iNeighborTa ];
						if ( Tb.IsValid() && Tc.IsValid() && Tb.AdjacentTo( Tc ) ) {


							int sharedFaceT_Tb = Tb.GetFaceFromVertices( sharedSegmentA, sharedSegmentB, p );
							assert( sharedFaceT_Tb >= 0 );							
							const int c = T.GetVertexOutsideFace( faceT );
							assert( Ta.GetVertexOutsideFace( faceTa ) == c );
							const int d = Tb.GetVertexOutsideFace( sharedFaceT_Tb );
#if _DEBUG
							{
								int ta0, ta1, ta2;
								Ta.GetFaceVertices( faceTa, ta0, ta1, ta2 );
								assert( Tc.GetVertexOutsideFace( Tc.GetFaceFromVertices( ta0, ta2, ta1 ) ) == d );
							}
#endif
							// Flip44 can be symmetrical around the edge sharedSegmentA <-> sharedSegmentB, so the flat plane
							// can be [ ssA, ssB, c, d ] or it's 90deg rotation [ ssA, ssB, pD, pP ]
							if ( Coplanar( vertices[ sharedSegmentA ], vertices[ sharedSegmentB ], vertices[ c ], vertices[ d ] ) ) {
								unsigned int result[4];
								Flip44( iT, iTa, iNeighborT, iNeighborTa, tetrahedra, vertices, result );
								for( int i = 0; i < 4; i++ ) {
									needTesting.push( result[ i ] );
								}
							} else if ( Coplanar( vertices[ sharedSegmentA ], vertices[ sharedSegmentB ], pD, pP ) ) {
								unsigned int result[4];
								Flip44( iT, iNeighborT, iTa, iNeighborTa, tetrahedra, vertices, result );
								for( int i = 0; i < 4; i++ ) {
									needTesting.push( result[ i ] );
								}
							}
						}
					}
				}
				break;

			default:
				// something went wrong here
				assert( false );
				return;
		}
	}

	/*
	================
	Delaunay3D::InsertOnePoint
	================
	*/
	void Delaunay3D::InsertOnePoint( const CoreLib::List< Point >& vertices, const int pointIndex, 
									 CoreLib::List< tetrahedron_t >& tetrahedra ) {
		
		// Find the tetrahedron containing pointIndex
		int t = Walk( vertices[ pointIndex ], 0, vertices, tetrahedra );
		if ( t < 0 ) {
			assert( false );
			return;
		}
		
		// Insert pointIndex into t using a flip14
		unsigned int result[4];
		Flip14( pointIndex, t, tetrahedra, vertices, result );

		std::stack< unsigned int > stack;
		for( int i = 0; i < 4; i++ ) {
			stack.push( result[ i ] );
		}

		while( !stack.empty() ) {
#if _DEBUG && 0
			{
				for( int i  = 0; i < (int)stack.size() - 1; i++ ) {
					for( unsigned int j = i + 1 ; j < stack.size(); j++ ) {
						assert( stack._Get_container()[ i ] != stack._Get_container()[ j ] );		
					}
				}
			}
#endif
			// T = { a, b, c, p }
			const int iT = stack.top();
			stack.pop();
			tetrahedron_t& T = tetrahedra[ iT ];
			if ( !T.IsValid() ) { // might have been invalidated by another flip
				continue;
			}
					
			// T = { a, b, c, p }
			int face = 0;
			for( ; face < 4; face++ ) {
				if ( T.GetVertexOutsideFace( face ) == pointIndex ) {
					break;
				}
			}
			if ( face == 4 ) {
				continue; // something went wrong. skip the tetrahedron
			}

			assert( T.GetVertexOutsideFace( face ) == pointIndex );

			// Get adjacent tetrahedron Ta having a,b,c as a facet
			if ( T.neighbors[ face ] >= 0 ) {
				const int iTa = T.neighbors[ face ];
				tetrahedron_t& Ta = tetrahedra[ iTa ];			
				assert( Ta.IsValid() );

				int a,b,c;
				T.GetFaceVertices( face, a, b, c );
				const int sharedFace = Ta.GetFaceFromVertices( a, c, b ); // reverse face
				const int opposedVertex = Ta.GetVertexOutsideFace( sharedFace );

				// OpposedVertex CAN be pointIndex in a degenerate Case 1, where two adjacent tetrahedra also share
				// the vertex opposed to their shared face (that is, they're effectively the same tetrahedron with the 
				// faces declared in a different order). This can happen after inserting a vertex in the face of an existing
				// tetrahedron and causing N chained Case 4 till we close the cycle and reach the original point again.
				// This degenerate Case1 will generate 3 new (smaller) flat tetrahedra, that should eventually go away
				// with further flips
				// ! don't --> assert( opposedVertex != pointIndex );

				const Point& d = vertices[ opposedVertex ];
				
				// If d is inside the circumference of T then flip 
				const Point& T0 = vertices[ T.v[ 0 ] ];
				const Point& T1 = vertices[ T.v[ 2 ] ];
				const Point& T2 = vertices[ T.v[ 1 ] ];
				const Point& T3 = vertices[ T.v[ 3 ] ];							

				const bool flip =	T.IsFlat( vertices ) || // if our tetrahedron is flat, it is not valid and we must flip it
									( Orient( T0, T1, T2, T3 ) >= 0 && InSphere( T0, T1, T2, T3, d ) > 0 ) || 
									( Orient( T0, T2, T1, T3 ) >= 0 && InSphere( T0, T2, T1, T3, d ) > 0 );				
				if ( flip ) {
					Flip( iT, iTa, pointIndex, vertices, tetrahedra, stack );
				}
			}
		}
	}

	int intCompare( const void* p0, const void* p1 ) {
		return *(static_cast< const int* >(p0)) - *(static_cast< const int* >(p1));  
	}

} // namespace Delaunay3D
} // namespace Geometry
} // namespace RenderLib
