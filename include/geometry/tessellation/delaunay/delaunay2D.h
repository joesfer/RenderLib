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
#include <assert.h>
#include <stack>
#include <math/algebra/point/point2.h>
#include <math/algebra/vector/vector2.h>
#include <math/algebra/matrix/matrix3.h>
#include <math/algebra/matrix/matrix4.h>

#define DEBUG_STEPS 0

namespace RenderLib {
namespace Geometry {
namespace Delaunay2D {

	// TODO: move all Delaunay2D namespace into private static methods of a Delaunat2D class
	// TODO: analyze whether it makes sense to expose Adjacency class or it should remain private within Delaunay2D

#define LIST( type ) CoreLib::List< type, CoreLib::Memory::StaticMemoryPool< type > >

		const float POINT_ON_SEGMENT_DISTANCE_EPSILON	= 1e-4f;
		const float POINT_ON_SEGMENT_PARAMETRIC_EPSILON = 1e-5f;
		const float INSIDE_CIRCUMCIRCLE_EPSILON			= 1e-2f;
		const float COINCIDENT_POINTS_DISTANCE_EPSILON	= 1e-1f;

		template< typename T >
		class Adjacency {
		public:
			Adjacency() {
				vertices.setGranularity( 1024 );
				triangles.setGranularity( 1024 );
				edges.setGranularity( 1024 );
			}
			int CreateTriangle( unsigned int a, unsigned int b, unsigned int c ) {
				using namespace RenderLib::Math;
				assert( a != b && a != c && b != c );
#if _DEBUG
				for( size_t i = 0; i < triangles.size(); i++ ) {					
					assert( !( triangles[ i ].valid && triangles[ i ].Contains( a, b, c ) ) );
				}

				Vector3<T> AB( vertices[ b ].x - vertices[ a ].x, vertices[ b ].y - vertices[ a ].y, 0 );
				Vector3<T> AC( vertices[ c ].x - vertices[ a ].x, vertices[ c ].y - vertices[ a ].y, 0 );
				AB.normalize();
				AC.normalize();
				assert( Vector3<T>::cross( AB, AC ).z > 0 );
#endif
				int triangleIndex;
				if ( invalidTriangles.empty() ) {
					triangleIndex = triangles.size();
					triangles.append();
				} else {
					triangleIndex = invalidTriangles[ 0 ];
					invalidTriangles.removeIndexFast( 0 );
				}
				Triangle_t& triangle = triangles[ triangleIndex ];
				triangle.vertices[ 0 ] = a;
				triangle.vertices[ 1 ] = b;
				triangle.vertices[ 2 ] = c;

				if ( !vertexTriangleAdjacency.empty() ) {
					vertexTriangleAdjacency[ a ].append( triangleIndex );
					vertexTriangleAdjacency[ b ].append( triangleIndex );
					vertexTriangleAdjacency[ c ].append( triangleIndex );
				}


				for( int i = 0; i < 3; i++ ) {
					int e = FindEdge( triangle.vertices[ i ], triangle.vertices[ (i + 1) % 3 ] );
					if( e >= 0 ) {
						Edge_t& edge = edges[ e ];
						int edgeTriIdx = edge.vertices[ 0 ] == triangle.vertices[ i ] ? 0 : 1;
						assert( edge.triangles[ edgeTriIdx ] < 0 );
						edge.triangles[ edgeTriIdx ] = triangleIndex;
						triangle.edges[ i ] = edgeTriIdx == 0 ? e + 1 : -(e + 1);
					} else {
						e = CreateEdge( triangle.vertices[ i ], triangle.vertices[ ( i + 1 ) % 3 ] );
						Edge_t& edge = edges[ e ];
						edge.triangles[ 0 ] = triangleIndex;
						triangle.edges[ i ] = e + 1;
					}					
				}
				
				triangle.valid = true;
				return triangleIndex;
			}
			void RemoveTriangle( unsigned int t )
			{
				Triangle_t& triangle = triangles[ t ];
				assert( triangle.valid );

				for( int i = 0; i < 3; i++ ) {
					const int edgeIndex = triangle.edges[ i ];
					Edge_t& edge = edges[ abs( edgeIndex ) - 1 ];
					if ( edgeIndex < 0 ) {
						assert( edge.triangles[ 1 ] == t );
						edge.triangles[ 1 ] = -1;
					} else {
						assert( edge.triangles[ 0 ] == t );
						edge.triangles[ 0 ] = -1;
					}
					/*if ( edge.triangles[ 0 ] < 0 && edge.triangles[ 1 ] < 0 ) {
						// no longer used
						// we should remove the edge, but if we do, we'll invalidate references to other edges
					}*/

				}

				for( size_t i = 0; i < vertexTriangleAdjacency.size(); i++ ) {
					LIST( int )& adjacents = vertexTriangleAdjacency[ i ];
					adjacents.removeFast( (int)t );
				}

				triangle.valid = false;
				invalidTriangles.append( t );
			}

			// returns the triangle index of the triangle containing p
			int PointInTriangle( const RenderLib::Math::Vector2<T>& p ) {	
				using namespace RenderLib::Math;
				for( size_t i = 0; i < triangles.size(); i++ ) {

					const Triangle_t& t = triangles[ i ];
					if ( !t.valid ) {
						continue;
					}

					bool hit = false;

					// use barycentric coordinates to determine whether the point is inside the triangle
					
					// http://steve.hollasch.net/cgindex/math/barycentric.html

					const Vector2<T>& v0 = vertices[ t.vertices[ 0 ] ];
					const Vector2<T>& v1 = vertices[ t.vertices[ 1 ] ];
					const Vector2<T>& v2 = vertices[ t.vertices[ 2 ] ];

					const T b0 =  (v1.x - v0.x) * (v2.y - v0.y) - (v2.x - v0.x) * (v1.y - v0.y);
					if ( b0 != 0 ) {
						const T b1 = ((v1.x - p.x) * (v2.y - p.y) - (v2.x - p.x) * (v1.y - p.y)) / b0; 
						const T b2 = ((v2.x - p.x) * (v0.y - p.y) - (v0.x - p.x) * (v2.y - p.y)) / b0;
						const T b3 = ((v0.x - p.x) * (v1.y - p.y) - (v1.x - p.x) * (v0.y - p.y)) / b0;

						hit = b1 >=0 && b2 >= 0 && b3 >=0;
					}
		
					if ( hit ) {
						return i;
					}
				}
				return -1;
			}
			int PointInTriangleEdge( const RenderLib::Math::Vector2<T>& p, const int t ) {

				using namespace RenderLib::Geometry;

				const Triangle_t& triangle = triangles[ t ];
				assert( triangle.valid );
				
				int closestEdge = -1;
				T closestDistance = FLT_MAX;
				for( int i = 0; i < 3; i++ ) {
					assert( triangle.edges[ i ] != INT_MAX );
					int edgeIdx = abs( triangle.edges[ i ] ) - 1;
					const Edge_t& edge = edges[ edgeIdx ];
					float distance;
					if ( PointOnSegment( vertices[ edge.vertices[ 0 ] ],
										 vertices[ edge.vertices[ 1 ] ], 
										 p, 
										 Delaunay2D::POINT_ON_SEGMENT_DISTANCE_EPSILON,
										 Delaunay2D::POINT_ON_SEGMENT_PARAMETRIC_EPSILON, 
										 &distance ) ) {

						if ( distance < closestDistance ) {
							closestDistance = distance;
							closestEdge =  edgeIdx;
						}
					}
				}

				return closestEdge;
			}
	
			void SplitEdge( int edgeIndex, const RenderLib::Math::Vector2<T>& p, int result[ 4 ] ) {
				Edge_t& edge = edges[ edgeIndex ];
				assert( RenderLib::Geometry::PointOnSegment( vertices[ edge.vertices[ 0 ] ],
															vertices[ edge.vertices[ 1 ] ], 
															p, 
															Delaunay2D::POINT_ON_SEGMENT_DISTANCE_EPSILON, 
															Delaunay2D::POINT_ON_SEGMENT_PARAMETRIC_EPSILON ) );

				/*
						   B = edge.v[0]	
				          /|\
				         / | \         T1 = A,B,C   T2 = D,C,B
				        /  |  \
				       /   |   \       T1' = A,B,P  T2' = D,C,P  T3' = A,P,C  T4' = B,D,P
				      /    |    \
					A ---- P ---- D
					  \    |    /
					   \   |   /
					    \  |  /
						 \ | /
						  \|/
						   C = edge.v[1]




				*/

				for( int idx = 0; idx < 4; idx++ ) {
					result[ idx ] = -1;
				}

				const int triIdx1 = edge.triangles[ 0 ];
				const int triIdx2 = edge.triangles[ 1 ];

				const int tri1edge = triIdx1 >= 0 ? triangles[ triIdx1 ].LocalEdgeIndex( edgeIndex ) : -1;
				assert( triIdx1 < 0 || tri1edge >= 0 );
				const int tri2edge = triIdx2 >= 0 ? triangles[ triIdx2 ].LocalEdgeIndex( edgeIndex ) : -1;
				assert( triIdx2 < 0 || tri2edge >= 0 );
				const int A = triIdx1 >= 0 ? VertexOutOfTriEdge( triIdx1, tri1edge ) : -1;
				assert( triIdx1 < 0 || A >= 0);
				const int D = triIdx2 >= 0 ? VertexOutOfTriEdge( triIdx2, tri2edge ) : -1;
				assert( triIdx2 < 0 || D >= 0);

				if ( A < 0 || D < 0 ) return;

				const int B = edge.vertices[ 0 ];
				const int C = edge.vertices[ 1 ];

				assert( B >= 0 && C >= 0 );
				if ( B < 0 || C < 0 ) return;

				assert( triangles[ triIdx1 ].Contains( A, B, C ) );
				assert( triangles[ triIdx2 ].Contains( C, B, D ) );
				if ( triIdx1 >= 0 ) RemoveTriangle( triIdx1 );
				if ( triIdx2 >= 0 ) RemoveTriangle( triIdx2 );
				assert( edge.triangles[ 0 ] < 0 && edge.triangles[ 1 ] < 0 );

				int newVertex = vertices.append( p );
				//edge.vertices[ 1 ] = newVertex;
				
				int idx = 0;
				if ( A >= 0 && B >=0 ) {
					result[ idx++ ] = CreateTriangle( A, B, newVertex );
				}
				if ( D >= 0 && C >= 0 ) {
					result[ idx++ ] = CreateTriangle( D, C, newVertex );
				}
				if ( A >= 0 && C >= 0 ) {
					result[ idx++ ] = CreateTriangle( A, newVertex, C );
				}
				if ( B >= 0 && D >= 0 ) {
					result[ idx++ ] = CreateTriangle( B, D, newVertex );
				}				
			}


			void SplitTriangle( int triIdx, const RenderLib::Math::Vector2<T>& p, int result[ 3 ] ) {
				int newVertex = vertices.append( p );
				
				const Triangle_t& oldTri = triangles[ triIdx ];
				assert( oldTri.valid );

				const int A = oldTri.vertices[ 0 ];
				const int B = oldTri.vertices[ 1 ];
				const int C = oldTri.vertices[ 2 ];
				assert( A != B && A != C && A != newVertex && B != C && B != newVertex && C != newVertex );

				RemoveTriangle( triIdx );

				result[ 0 ] = CreateTriangle( A, B, newVertex );
				result[ 1 ] = CreateTriangle( B, C, newVertex );
				result[ 2 ] = CreateTriangle( C, A, newVertex );
			}


			int AdjacentTriangle( int triIndex, int edgeIndex ) const {
				const Triangle_t triangle = triangles[ triIndex ];
				assert( triangle.valid );
				assert( edgeIndex >=0 && edgeIndex < 3 );

				const int edgeGlobalIdx = abs( triangle.edges[ edgeIndex ] ) - 1;
				const Edge_t& edge = edges[ edgeGlobalIdx ];
				if ( triangle.edges[ edgeIndex ] < 0 ) {
					assert( edge.triangles[ 1 ] == triIndex );
					return edge.triangles[ 0 ];
				} else {
					assert( edge.triangles[ 0 ] == triIndex );
					return edge.triangles[ 1 ];
				}
			}	

			int VertexOutOfTriEdge( int triIndex, int edgeIndex ) const {
				const Triangle_t triangle = triangles[ triIndex ];
				assert( triangle.valid );
				assert( edgeIndex >=0 && edgeIndex < 3 );

				const int globalEdgeIdx = abs( triangle.edges[ edgeIndex ] ) - 1;
				const Edge_t& edge = edges[ globalEdgeIdx ];				
				if ( edge.Links( triangle.vertices[ 0 ], triangle.vertices[ 1 ] ) ) {
					return triangle.vertices[ 2 ];
				} else if ( edge.Links( triangle.vertices[ 0 ], triangle.vertices[ 2 ] ) ) {
					return triangle.vertices[ 1 ];
				} else {
					assert( edge.Links( triangle.vertices[ 1 ], triangle.vertices[ 2 ] ) );
					return triangle.vertices[ 0 ];
				}
			}

			bool FlipTriangles( int tri1, int tri2, int result[2] ) {

				using namespace RenderLib::Geometry;

				Triangle_t& triangle1 = triangles[ tri1 ];
				assert( triangle1.valid );
				Triangle_t& triangle2 = triangles[ tri2 ];
				assert( triangle2.valid );

				int commonEdgeIdx = CommonEdge( triangle1, triangle2 );
				assert( commonEdgeIdx >= 0 );
				Edge_t& commonEdge = edges[ commonEdgeIdx ];
				assert( commonEdge.triangles[ 0 ] == tri1 || commonEdge.triangles[ 0 ] == tri2 );
				assert( commonEdge.triangles[ 1 ] == tri1 || commonEdge.triangles[ 1 ] == tri2 );

				// Tri1 = A,B,C
				// Tri2 = B,D,C
				// commonEdge = B,C

				const int localEdgeT1 = triangle1.LocalEdgeIndex( commonEdgeIdx );
				assert( localEdgeT1 >= 0 );
				const int localEdgeT2 = triangle2.LocalEdgeIndex( commonEdgeIdx );
				assert( localEdgeT2 >= 0 );

				int A = VertexOutOfTriEdge( tri1, localEdgeT1 );
				int D = VertexOutOfTriEdge( tri2, localEdgeT2 );
				assert( A >= 0 );
				assert( D >= 0 );

				int B = ( commonEdge.triangles[ 0 ] == tri1 ? commonEdge.vertices[ 0 ] : commonEdge.vertices[ 1 ] );
				int C = ( B == commonEdge.vertices[ 0 ] ? commonEdge.vertices[ 1 ] : commonEdge.vertices[ 0 ] );

				assert( A != B && A != C && A != D && B != C && B != D && C != D );

				if ( !SegmentIntersect( vertices[ A ], vertices[ D ], vertices[ B ], vertices[ C ] ) ) {
					// can't flip
					return false;
				}

				if ( false && !IsWorthFlipping(  A, B, C, D ) ) {
					// The resulting triangles would not improve the triangulation, don't bother
					return false;
				}
				const T closestAngleBefore = __min( ClosestAngleOnTri( A, B, C ), ClosestAngleOnTri( A, D, C ) );
				
				//assert( triangles[ tri1 ].InsideCircumcircle( vertices[ D ], vertices ) > Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON );
				//assert( triangles[ tri2 ].InsideCircumcircle( vertices[ A ], vertices ) > Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON);
			
				RemoveTriangle( tri1 );
				RemoveTriangle( tri2 );

				result[ 0 ] = CreateTriangle( A, B, D );
				result[ 1 ] = CreateTriangle( A, D, C );

#if _DEBUG && 0
				FILE* fd = fopen("C:/Users/Jose/Documents/Programming/triFlip.txt", "wt");
				if ( fd ) {
					const RenderLib::Point2<T> vA = vertices[ A ];
					const RenderLib::Point2<T> vB = vertices[ B ];
					const RenderLib::Point2<T> vC = vertices[ C ];
					const RenderLib::Point2<T> vD = vertices[ D ];
					RenderLib::Point2<T> center;
					T radius;
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vA.x, vA.y, vB.x, vB.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vB.x, vB.y, vC.x, vC.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vA.x, vA.y, vC.x, vC.y );
					CircleFrom3Points<T>( vA, vB, vC, center, radius );
					fprintf( fd, "circle -r %f -centerX %f -centerY %f;", radius, center.x, center.y );

					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vD.x, vD.y, vB.x, vB.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vD.x, vD.y, vC.x, vC.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vB.x, vB.y, vC.x, vC.y );
					CircleFrom3Points<T>( vB, vD, vC, center, radius );
					fprintf( fd, "circle -r %f -centerX %f -centerY %f;", radius, center.x, center.y );

					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vA.x, vA.y, vB.x, vB.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vB.x, vB.y, vD.x, vD.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vA.x, vA.y, vD.x, vD.y );
					CircleFrom3Points<T>( vA, vB, vD, center, radius );
					fprintf( fd, "circle -r %f -centerX %f -centerY %f;", radius, center.x, center.y );

					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vD.x, vD.y, vA.x, vA.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vD.x, vD.y, vC.x, vC.y );
					fprintf( fd, "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", vA.x, vA.y, vC.x, vC.y );
					CircleFrom3Points<T>( vA, vD, vC, center, radius );
					fprintf( fd, "circle -r %f -centerX %f -centerY %f;", radius, center.x, center.y );

					fclose(fd);
				}

#endif

				const T closestAngleAfter = __min( ClosestAngleOnTri( A, B, D ), ClosestAngleOnTri( A, D, C ) );
				
				//assert( closestAngleBefore <= closestAngleAfter );
#if _DEBUG
				//if ( closestAngleBefore < closestAngleAfter ) { // if the angles are the same it means both triangles have their vertices on the same circumcircle
				//	assert( triangles[ result[ 0 ] ].InsideCircumcircle( vertices[ C ], vertices ) <= Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON );
				//	assert( triangles[ result[ 1 ] ].InsideCircumcircle( vertices[ B ], vertices ) <= Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON );
				//}
#endif
				return true;
			}
			
			bool IsWorthFlipping( int A, int B, int C, int D ) const {
				const T closestAngleBefore = __min( ClosestAngleOnTri( A, B, C ), ClosestAngleOnTri( A, D, C ) );
				const T closestAngleAfter = __min( ClosestAngleOnTri( A, B, D ), ClosestAngleOnTri( A, D, C ) );

				return closestAngleBefore < closestAngleAfter;
			}

			T ClosestAngleOnTri( int vA, int vB, int vC ) const {
				using namespace RenderLib::Math;
				int indices[3] = { vA, vB, vC };				

				T closestAngle = static_cast<T>(RenderLib::Math::PI);
				for( int i = 0; i < 3; i++ ) {
					const Vector2<T>& nAB = Vector2<T>::normalize( Vector2<T>( vertices[ indices[ ( i + 1 ) % 3 ] ] - vertices[ indices[ i ] ] ) );
					const Vector2<T>& nAC = Vector2<T>::normalize( Vector2<T>( vertices[ indices[ ( i + 2 ) % 3 ] ] - vertices[ indices[ i ] ] ) );
					const T angle = acos( Vector2<T>::dot(nAB, nAC) );
					if ( angle < closestAngle ) {
						closestAngle = angle;
					}
				}
				return closestAngle;
			}

			bool BuildVertexTriangleAdjacency() {
				if ( triangles.empty() ) {
					return false;
				}

				vertexTriangleAdjacency.resize( vertices.size() );
				for( size_t t = 0; t < triangles.size(); t++ ) {
					Triangle_t& tri = triangles[ t ];
					for( int v = 0; v < 3; v++ ) {
						vertexTriangleAdjacency[ tri.vertices[ v ] ].append( t );
					}
				}

#if _DEBUG
				for( size_t i = 0; i < vertexTriangleAdjacency.size(); i++ ) {
					LIST( int )& adjacents = vertexTriangleAdjacency[ i ];
					for( size_t j = 0; j < adjacents.size(); j++ ) {
						Triangle_t& t = triangles[ adjacents[ j ] ];
						assert( t.Contains( i ) );
					}
				}
#endif
				return true;
			}

			unsigned int DeleteTrianglesIntersectingSegment_r( const int vIdx0, const int vIdx1, 
																LIST( int )& upperVertices,
																LIST( int )& lowerVertices ) {

				using namespace RenderLib::Geometry;
				using namespace RenderLib::Math;

				if ( vIdx0 < 0 || vIdx1 < 0 || (size_t)vIdx0 >= vertices.size() || (size_t)vIdx1 >= vertices.size() ) {
					return 0;
				}

				
				int tri = -1;
				int triEdge = -1;
				const Vector2<T>& v0 = vertices[ vIdx0 ];
				const Vector2<T>& v1 = vertices[ vIdx1 ];
				const triList_t& v0Adjacents = vertexTriangleAdjacency[ vIdx0 ];

				// find the triangle containing v0
				for( size_t t = 0; t < v0Adjacents.size(); t++ ) {
					const Triangle_t& triangle = triangles[ v0Adjacents[ t ] ];
					assert( triangle.Contains( vIdx0 ) );
					for( int e = 0; e < 3; e++ ) {
						const int edgeIdx = abs( triangle.edges[ e ] ) - 1;
						Edge_t edge = edges[ edgeIdx ];
						if ( edge.vertices[ 0 ] == vIdx0 || edge.vertices[ 1 ] == vIdx0 ) {

							if ( edge.vertices[ 0 ] == vIdx1 || edge.vertices[ 1 ] == vIdx1 ) {
								// there's no need to insert anything: the initial segment
								// already existed as a collection of subsegments that we
								// traversed in recursive calls to DeleteTrianglesIntersectingSegment
								return true;
							}

							// check for degenerate cases, where the segment v0-v1 is parallel
							// to a triangle edge. All we need to do in this case is instead 
							// inserting the segment from the parallel edge endpoint to v1

							const int other = edge.vertices[ 0 ] == vIdx0 ? edge.vertices[ 1 ] : edge.vertices[ 0 ];

							if ( PointOnSegment( v0, v1, vertices[ other ], 1e-3f, 0.f ) ) {
								return DeleteTrianglesIntersectingSegment_r( other, vIdx1, upperVertices, lowerVertices );			
							}

							continue;
						}

						const Vector2<T>& triVi = vertices[ edge.vertices[ 0 ] ];
						const Vector2<T>& triVj = vertices[ edge.vertices[ 1 ] ];
						
						if ( PointOnSegment( v0, v1, triVi, 1e-3f, 0.f ) ||
							 PointOnSegment( v0, v1, triVj, 1e-3f, 0.f ) ) {
							continue;
						}
						// we now for certain that the segment is not parallel

						if ( SegmentIntersect( v0, v1, triVi, triVj ) ) {
							triEdge = e;
							tri = v0Adjacents[ t ];

							upperVertices.append( vIdx0 );
							lowerVertices.append( vIdx0 );

							if ( SegmentVertexSide( v0, v1, triVi ) >= 0 ) {
								upperVertices.append( edge.vertices[ 0 ] );
								assert( SegmentVertexSide( v0, v1, triVj ) < 0 );
								lowerVertices.append( edge.vertices[ 1 ] );
							} else {
								lowerVertices.append( edge.vertices[ 0 ] );
								assert( SegmentVertexSide( v0, v1, triVj ) >= 0 );
								upperVertices.append( edge.vertices[ 1 ] );
							}

							break;
						}
					}
					if ( tri >= 0 ) {
						break;
					}
				}
				if ( tri < 0 || triEdge < 0 ) {
					return 0;
				}				

				LIST( int ) toDelete;
				do {
					int next = this->AdjacentTriangle( tri, triEdge );
					assert( next >= 0 );	
					if ( next < 0 ) {
						break;
					}

					const int globalEdgeJustCrossed = abs( triangles[ tri ].edges[ triEdge ] ) - 1;
					toDelete.append( tri );
					tri = next;

					const Triangle_t& triangle = triangles[ tri ];
					const int localEdgeJustCrossed = triangle.LocalEdgeIndex( globalEdgeJustCrossed );
					assert( localEdgeJustCrossed >= 0 );

					int vNext = this->VertexOutOfTriEdge( tri, localEdgeJustCrossed );
					if ( vNext == vIdx1 ) {
						upperVertices.append( vNext );
						lowerVertices.append( vNext );
					} else {
						if ( PointOnSegment( v0, v1, vertices[ vNext ] ) ) {
							// vNext lies on the v0-v1 segment, dividing it into two
							// so we'll now process the first half and make a recursive call
							// to take care of the remaining half

							DeleteTrianglesIntersectingSegment_r( vNext, vIdx1, upperVertices, lowerVertices );

							break;
						}
						if ( SegmentVertexSide( v0, v1, vertices[ vNext ] ) >= 0 ) {
							upperVertices.append( vNext );
						} else {
							lowerVertices.append( vNext );
						}

						triEdge = -1;
						
						for( int e = 0; e < 3; e++ ) {
							if ( e == localEdgeJustCrossed ) {
								// this will surely intersect the v0-v1 segment, but it's
								// the segment through which we come from, so skip it
	#if _DEBUG
								const int edgeIdx = abs( triangle.edges[ e ] ) - 1;
								Edge_t edge = edges[ edgeIdx ];
								const Vector2<T>& triVi = vertices[ edge.vertices[ 0 ] ];
								const Vector2<T>& triVj = vertices[ edge.vertices[ 1 ] ];
								assert( SegmentIntersect( v0, v1, triVi, triVj ) ); 
	#endif
								continue;
							}
							const int edgeIdx = abs( triangle.edges[ e ] ) - 1;
							Edge_t edge = edges[ edgeIdx ];
							const Vector2<T>& triVi = vertices[ edge.vertices[ 0 ] ];
							const Vector2<T>& triVj = vertices[ edge.vertices[ 1 ] ];
							if ( SegmentIntersect( v0, v1, triVi, triVj, 0 ) ) {
								triEdge = e;			
								assert( edge.vertices[ 0 ] == vNext || edge.vertices[ 1 ] == vNext );
								break;
							}
						}

						assert( triEdge >= 0 );
						if ( triEdge < 0 ) {
							return false;
						}
					}

				} while( !triangles[ tri ].Contains( vIdx1 ) );
		
				toDelete.append( tri );


#if _DEBUG && DEBUG_STEPS
				// dump to maya
				{			
					{
						Vector2f& a = vertices[ vIdx0 ];
						Vector2f& b = vertices[ vIdx1 ];
						MString cmd = CoreLib::varArgsStr<1024>( "$c = `curve -d 1 -p %f %f 0 -p %f %f 0 -k 0 -k 1`;", a.x, a.y, b.x, b.y );
						MGlobal::executeCommand( cmd );
					}

					MPointArray verts;
					for( size_t i = 0; i < vertices.size(); i++ ) {
						MPoint v( vertices[ i ].x, vertices[ i ].y, 0 );
						verts.append( v );
					}

					MIntArray triangleVertices;
					for( size_t i = 0; i < toDelete.size(); i++ ) {
						const Adjacency<T>::Triangle_t& triangle = triangles[ toDelete[ i ] ];
						if ( !triangle.valid ) {
							continue;
						}
						triangleVertices.append( triangle.vertices[ 0 ] );
						triangleVertices.append( triangle.vertices[ 1 ] );
						triangleVertices.append( triangle.vertices[ 2 ] );
					}


					const int resultingTris = triangleVertices.length() / 3;
					MFnMeshData meshData;
					MObject meshObj = meshData.create();
					MFnMesh mesh( meshObj );
					MIntArray polygonCounts( resultingTris, 3 );
					MStatus status;
					mesh.create( verts.length(), resultingTris, verts, polygonCounts, triangleVertices, MObject::kNullObj, &status );
					MGlobal::addToModel( meshObj );			
				}
#endif
				for( size_t i = 0; i < toDelete.size(); i++ ) {
					RemoveTriangle( toDelete[ i ] );
				}

				// sort the upper/lower lists in the correct order
				if ( upperVertices.size() > 2 ) {
					const Vector2<T> v1 = vertices[ upperVertices[ 1 ] ] - vertices[ upperVertices[ 0 ] ];
					const Vector2<T> v2 = vertices[ upperVertices[ upperVertices.size() - 1 ] ] - vertices[ upperVertices[ 0 ] ];
					const float crossProductZ = (v1.x * v2.y) - (v1.y * v2.x);
					const bool reversed = crossProductZ < 0;
					if ( reversed ) {
						int aux;
						for( size_t i = 0; i < upperVertices.size() / 2; i++ ) {
							aux = upperVertices[ i ];
							upperVertices[ i ] = upperVertices[ upperVertices.size() - 1 - i ];
							upperVertices[ upperVertices.size() - 1 - i ] = aux;
						}
					}
				}
				if ( lowerVertices.size() > 2 ) {
					const Vector2<T> v1 = vertices[ lowerVertices[ 1 ] ] - vertices[ lowerVertices[ 0 ] ];
					const Vector2<T> v2 = vertices[ lowerVertices[ lowerVertices.size() - 1 ] ] - vertices[ lowerVertices[ 0 ] ];
					const float crossProductZ = (v1.x * v2.y) - (v1.y * v2.x);
					const bool reversed = crossProductZ < 0;
					if ( reversed ) {
						int aux;
						for( size_t i = 0; i < lowerVertices.size() / 2; i++ ) {
							aux = lowerVertices[ i ];
							lowerVertices[ i ] = lowerVertices[ lowerVertices.size() - 1 - i ];
							lowerVertices[ lowerVertices.size() - 1 - i ] = aux;
						}
					}
				}
				return toDelete.size();
			}

			bool TriangulateSubPolygon_r( const LIST( int )& indices ) {

				using namespace RenderLib::Geometry;
				using namespace RenderLib::Math;

				if ( indices.size() < 3 ) {
					return false;
				}

				if ( indices.size() == 3 ) {
					// trivial triangulation
					return CreateTriangle( indices[ 0 ], indices[ 1 ], indices[ 2 ] ) >= 0;
				}

				// Split subpolygon

				// look for a candidate point P between indices 1 and N-2
				// so that the circumference passing through 0, p, N-1 is empty
				Vector2<T>& a = vertices[ indices[ 0 ] ];
				Vector2<T>& b = vertices[ indices[ indices.size() - 1 ] ];
				int splitPoint = -1;
				for( int i = 1; i < (int)indices.size() - 1; i++ ) {
					Vector2<T>& p = vertices[ indices[ i ] ];
					bool empty = true;
					for( int j = 1; j < (int)indices.size() - 1; j++ ) {
						if ( j == i ) {
							continue;
						}

						if ( InCircle( a, p, b, vertices[ indices[ j ] ] ) > 0 ) {
							empty = false;
							break;
						}
					}
					if ( empty ) {
						splitPoint = (int)i;
						break;
					}
				}

				if ( splitPoint <= 0 ) {					
					assert( false );
					return false;
				
				}

				LIST( int ) left, right;
				for( int i = 0; i <= splitPoint; i++ ) {
					left.append( indices[ i ] );
				}			
				
				for( size_t i = (size_t)splitPoint; i < indices.size(); i++ ) {
					right.append( indices[ i ] );
				}
				assert( left.size() < indices.size() );
				assert( right.size() < indices.size() );
				if ( !PointOnSegment<T>( vertices[ indices[ 0 ] ], vertices[ indices[ indices.size() - 1 ] ], vertices[ indices[ splitPoint ] ] ) ) {
					CreateTriangle( indices[ 0 ], indices[ splitPoint ], indices[ indices.size() - 1 ] ); 
				}
				
				if ( left.size() >= 3 ) {
					if ( !TriangulateSubPolygon_r( left ) ) {
						return false;
					}
				}
				if ( right.size() >= 3 ) {
					if ( !TriangulateSubPolygon_r( right ) ) {
						return false;
					}
				}
				return true;
			}

			int FindEdge( int v1, int v2 ) const {
				assert( v1 != v2 );
				for( size_t i = 0; i < edges.size(); i++ ) {
					if ( edges[ i ].Links( v1, v2 ) )
					{
						return (int)i;
					}
				}
				return -1;
			}

			float SegmentVertexSide( const RenderLib::Math::Vector2<T>& A, 
									 const RenderLib::Math::Vector2<T>& B, 
									 const RenderLib::Math::Vector2<T>& P ) const {
				/*const Vector2<T> AB = B - A;
				const Vector2<T> ABperp( AB.y, -AB.x );
				const Vector2<T> AP = P - A;
				return ABperp * AP;*/
				return RenderLib::Geometry::Signed2DTriangleArea( A, B, P );
			}

			static T Orient( const RenderLib::Math::Vector2<T>& a, 
							 const RenderLib::Math::Vector2<T>& b, 
							 const RenderLib::Math::Vector2<T>& c ) {
				using namespace RenderLib::Math;
				return Matrix3<T>( a.x, a.y, 1, 
								   b.x, b.y, 1,
								   c.x, c.y, 1 ).determinant();
			}

			// returns > 0 if p is inside the circle described by A,B,C < 0 outside, = 0 on the circle			
			static T InCircle(	const RenderLib::Math::Vector2<T>& a, 
								const RenderLib::Math::Vector2<T>& b, 
								const RenderLib::Math::Vector2<T>& c, 
								const RenderLib::Math::Vector2<T>& p ) {
				using namespace RenderLib::Math;
				const T det = Matrix4<T>( a.x, a.y, a.x * a.x + a.y * a.y, 1, 
										  b.x, b.y, b.x * b.x + b.y * b.y, 1,
										  c.x, c.y, c.x * c.x + c.y * c.y, 1,
										  p.x, p.y, p.x * p.x + p.y * p.y, 1 ).determinant();
				if ( Orient( a, b, c ) > 0.f ) {
					return det;
				} else {
					return -det;
				}
			}
		public:
			struct Edge_t {
				int vertices[2]; // endpoints
				int triangles[2]; // triangles on the positive side [0], and negative side [1]

				Edge_t() {
					vertices[ 0 ] = vertices[ 1 ] = -1;
					triangles[ 0 ] = triangles[ 1 ] = -1;
				}

				inline bool Links( int v1, int v2 ) const {
					assert( v1 != v2 );
					return ( vertices[ 0 ] == v1 && vertices[ 1 ] == v2 ) || ( vertices[ 0 ] == v2 && vertices[ 1 ] == v1 );
				}
			};
			struct Triangle_t {
				int vertices[3];
				int edges[3]; // signed, 1-based
				bool valid;
				RenderLib::Math::Point2f	cirumcircleCenter;
				T							circumcircleRadius;

				Triangle_t() {
					vertices[ 0 ] = vertices[ 1 ] = vertices[ 2 ] = -1;
					edges[ 0 ] = edges[ 1 ] = edges[ 2 ] = INT_MAX;
					valid = false;
					circumcircleRadius = -1;
				}
				
				inline bool Contains( int v ) const {
					return  ( vertices[ 0 ] == v || vertices[ 1 ] == v || vertices[ 2 ] == v );
				}

				inline bool Contains( int a, int b, int c ) const {
					return  Contains( a ) && Contains( b ) && Contains( c );
				}

				int LocalEdgeIndex( int globalEdgeIndex ) const {
					for( int i = 0; i < 3; i++ ) {
						if( abs( edges[ i ] ) - 1 == globalEdgeIndex ) {
							return i;
						}
					}
					return -1;
				}			

				T InsideCircumcircle( const RenderLib::Math::Vector2<T>& p, const LIST( RenderLib::Math::Vector2<T> )& verts ) const {
					using namespace RenderLib::Math;
					const Vector2<T>& A = verts[ vertices[ 0 ] ];
					const Vector2<T>& B = verts[ vertices[ 1 ] ];
					const Vector2<T>& C = verts[ vertices[ 2 ] ];
				
					const T det = InCircle( A, B, C, p );
#if _DEBUG && 0
					{
						Point2<T> center; 
						T radius = 0;
						if ( RenderLib::CircleFrom3Points<T>( A, B, C, center, radius ) ) {
							const T dist = center.DistanceTo( p );

							if( det < -Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON ) { 
								assert( dist > radius - radius * Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON );
							} else if ( det > Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON ) {
								assert( dist < radius + radius * Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON );
							} 			
						}
					}
#endif
					return det;
				}
			};			

		public:

			LIST( Edge_t )				edges;
			LIST( Triangle_t )			triangles;
			LIST( RenderLib::Math::Vector2<T> ) vertices;
			LIST( unsigned int )		invalidTriangles;
			
			typedef LIST( int ) triList_t;
			LIST( triList_t )	vertexTriangleAdjacency;

		private:
			unsigned int CreateEdge( int v1, int v2 ) {
				Edge_t& e = edges.append();
				e.vertices[ 0 ] = v1;
				e.vertices[ 1 ] = v2;
				e.triangles[ 0 ] = -1;
				e.triangles[ 1 ] = -1;
				return edges.size() - 1;
			}

			int CommonEdge( const Triangle_t& tri1, const Triangle_t& tri2 ) const {
				for( int i = 0; i < 3; i++ ) {
					const int edgeT1 = abs( tri1.edges[ i ] ) - 1;
					for( int j = 0; j < 3; j++ ) {
						const int edgeT2 = abs( tri2.edges[ j ] ) - 1;
						if ( edgeT1 == edgeT2 ) {
							return edgeT1;
						}
					}
				}
				return -1;
			}

		};

		

		template< typename T >
		bool Delaunay2D( const LIST( RenderLib::Math::Vector2<T> )& vertices, 
						 LIST(int)* outTriangles = NULL, 
						 Adjacency<T>* adjacencyInfo = NULL ) {

			/* Implements Lawson algorithm: 
			http://www.henrikzimmer.com/VoronoiDelaunay.pdf
			
			Unlike the Watson algorithm the Lawson algorithm is not based finding circumcircles and deleting
			triangles to obtain an insertion polygon. Lawson's incremental insertion algorithm utilizes edge
			flippings to achieve the same result and thus avoids a possibly faulty, degenerate mesh which can
			occur using Watsons method.
			In the same way as before, if we are not given a start triangulation we use a super triangle
			enclosing all vertices in V . In every insertion step a vertex p belonging to V is inserted. 
			A simple retriangulation is made where the edges are inserted between p and the corner vertices of 
			the triangle containing p (fig.11). For all new triangles formed the circumcircles are checked, if they contain
			any neighbouring vertex the edge between them is flipped. This process is executed recursively
			until there are no more faulty triangles and no more flips are required (fig.12). The algorithm then
			moves on, inserting another vertex from V until they have all been triangulated into a mesh, then,
			like before the super triangle and its edge are removed.
			*/

			if ( vertices.size() < 3 ) {
				return false;
			}

			Adjacency<T>* adjacency = adjacencyInfo;
			if ( adjacencyInfo == NULL ) {
				adjacency = new Adjacency<T>;
			};
			
			unsigned int stIdx1, stIdx2, stIdx3;
			Delaunay2DCreateSuperTriangle(vertices, adjacency, stIdx1, stIdx2, stIdx3 );


			if ( !Delaunay2DInsertPoints(vertices, adjacency) ) return false;


#if _DEBUG && DEBUG_STEPS
			// dump to maya
			{				
				MPointArray vertices;
				for( int i = 0; i < adjacency->vertices.Num(); i++ ) {
					MPoint v( adjacency->vertices[ i ].x, adjacency->vertices[ i ].y, 0 );
					vertices.append( v );
				}

				MIntArray triangleVertices;
				for( int i = 0; i < adjacency->triangles.Num(); i++ ) {
					const Adjacency<T>::Triangle_t& triangle = adjacency->triangles[ i ];
					if ( !triangle.valid ) {
						continue;
					}
					triangleVertices.append( triangle.vertices[ 0 ] );
					triangleVertices.append( triangle.vertices[ 1 ] );
					triangleVertices.append( triangle.vertices[ 2 ] );
				}


				const int resultingTris = triangleVertices.length() / 3;
				MFnMeshData meshData;
				MObject meshObj = meshData.create();
				MFnMesh mesh( meshObj );
				MIntArray polygonCounts( resultingTris, 3 );
				MStatus status;
				mesh.create( vertices.length(), resultingTris, vertices, polygonCounts, triangleVertices, MObject::kNullObj, &status );
				MGlobal::addToModel( meshObj );			
			}
#endif
			Delaunay2DRemoveSuperTriangle(adjacency, stIdx1, stIdx2, stIdx3, outTriangles, vertices);

#if _DEBUG && DEBUG_STEPS
			// dump to maya
			{				
				MPointArray vertices;
				for( int i = 0; i < adjacency->vertices.Num(); i++ ) {
					MPoint v( adjacency->vertices[ i ].x, adjacency->vertices[ i ].y, 0 );
					vertices.append( v );
				}

				MIntArray triangleVertices;
				for( int i = 0; i < adjacency->triangles.Num(); i++ ) {
					const Adjacency<T>::Triangle_t& triangle = adjacency->triangles[ i ];
					if ( !triangle.valid ) {
						continue;
					}
					triangleVertices.append( triangle.vertices[ 0 ] );
					triangleVertices.append( triangle.vertices[ 1 ] );
					triangleVertices.append( triangle.vertices[ 2 ] );
				}


				const int resultingTris = triangleVertices.length() / 3;
				MFnMeshData meshData;
				MObject meshObj = meshData.create();
				MFnMesh mesh( meshObj );
				MIntArray polygonCounts( resultingTris, 3 );
				MStatus status;
				mesh.create( vertices.length(), resultingTris, vertices, polygonCounts, triangleVertices, MObject::kNullObj, &status );
				MGlobal::addToModel( meshObj );			
			}
#endif
			if ( adjacencyInfo == NULL ) {
				delete adjacency;
			}

			return true;
		}

		template< typename T >
		void Delaunay2DRemoveSuperTriangle( Adjacency<T>* adjacency, 
											int stIdx1, 
											int stIdx2, 
											int stIdx3, 
											LIST(int)* outTriangles, 
											const LIST( RenderLib::Math::Vector2<T> )& vertices ) 
		{
			// remove triangles containing vertices from the supertriangle
			for( size_t i = 0; i < adjacency->triangles.size(); i++ ) {
				const Adjacency<T>::Triangle_t& triangle = adjacency->triangles[ i ];
				if ( !triangle.valid ) {
					continue;
				}
				if ( triangle.Contains( stIdx1 ) || triangle.Contains( stIdx2 ) || triangle.Contains( stIdx3 ) ) {
					adjacency->RemoveTriangle( i );
				} else {
					if ( outTriangles != NULL ) {
						for( int j = 0; j < 3; j++ ) {
							assert( triangle.vertices[ j ] >= 0 && triangle.vertices[ j ] < (int)adjacency->vertices.size() );
							outTriangles->append( triangle.vertices[ j ] );
						}
					}
				}
			}
			adjacency->invalidTriangles.clear();

			// remove first 3 vertices (belonging to the supertriangle) and remap all remaining tris
			if ( outTriangles != NULL ) {
				for( size_t i = 0; i < outTriangles->size(); i++ )
				{
					(*outTriangles)[i] -= 3;
				}
			}

			for( size_t i = 0; i < adjacency->triangles.size(); i++ ) {
				if ( !adjacency->triangles[ i ].valid ) {
					const int remappedTriangle = (int)adjacency->triangles.size() - 1;
					adjacency->triangles.removeIndexFast( i );

					// remap edges pointing to this triangle
					for( size_t e = 0; e < adjacency->edges.size(); e++ ) {
						Adjacency<T>::Edge_t& edge = adjacency->edges[ e ];
						if ( edge.triangles[ 0 ] == remappedTriangle ) {
							edge.triangles[ 0 ] = i;
						}
						if ( edge.triangles[ 1 ] == remappedTriangle ) {
							edge.triangles[ 1 ] = i;
						}
					}

					i--;
				} else {
					Adjacency<T>::Triangle_t& t = adjacency->triangles[ i ];
					t.vertices[ 0 ] -= 3;
					t.vertices[ 1 ] -= 3;
					t.vertices[ 2 ] -= 3;
				}
			}
			adjacency->vertices.resize( vertices.size(), false );
			for( size_t i = 0; i < vertices.size(); i++ ) {
				adjacency->vertices[ i ] = vertices[ i ];
			}
			for( size_t i = 0; i < adjacency->edges.size(); i++ ) {
				adjacency->edges[ i ].vertices[ 0 ] -= 3;
				adjacency->edges[ i ].vertices[ 1 ] -= 3;
			}
		}
		template< typename T >
		void Delaunay2DCreateSuperTriangle( const LIST( RenderLib::Math::Vector2<T> )& vertices, 
											Adjacency<T>* adjacency, 
											unsigned int& stIdx1, unsigned int &stIdx2, unsigned int& stIdx3 ) {
			// Calculate super triangle
			
			using namespace RenderLib::Geometry;
			using namespace RenderLib::Math;

			Bounds2D bounds;
			for( size_t i = 0; i < vertices.size(); i++ ) {
				bounds.expand( vertices[ i ] );
			}

			Point2f center;
			T radius;
			bounds.boundingCircunference( &center, &radius );

			Vector2<T>& st1 = adjacency->vertices.append();
			Vector2<T>& st2 = adjacency->vertices.append();
			Vector2<T>& st3 = adjacency->vertices.append();

			st1.x = center.x + 2.0f * radius * cosf( 0.f );
			st1.y = center.y + 2.0f * radius * sinf( 0.f );
			st2.x = center.x + 2.0f * radius * cosf( (float)DEG2RAD(120.0) );
			st2.y = center.y + 2.0f * radius * sinf( (float)DEG2RAD(120.0) );
			st3.x = center.x + 2.0f * radius * cosf( (float)DEG2RAD(240.0) );
			st3.y = center.y + 2.0f * radius * sinf( (float)DEG2RAD(240.0) );

			stIdx1 = adjacency->vertices.size() - 3;
			stIdx2 = adjacency->vertices.size() - 2;
			stIdx3 = adjacency->vertices.size() - 1;

			adjacency->CreateTriangle(	stIdx1, stIdx2, stIdx3 );
		}
		
		template< typename T >
		bool Delaunay2DInsertPoints( const LIST( RenderLib::Math::Vector2<T> )& vertices, 
									 Adjacency<T>* adjacency ) 
		{
			using namespace RenderLib::Math;

			// Insert points
			
			LIST( int ) toCheck;		

			for( size_t i = 0; i < vertices.size(); i++ ) {

				// Insert Vi
				const Vector2<T>& Vi = vertices[ i ];

				bool skip = false;
				for( size_t j = 0; j < adjacency->vertices.size(); j++ ) {
					if ( ( Vi - adjacency->vertices[ j ] ).length() <= Delaunay2D::COINCIDENT_POINTS_DISTANCE_EPSILON ){
						// the point has already been inserted. Skip it
						skip = true;
						break;
					}
				}
				if ( skip ) {
					//continue;
				}

				int tri = adjacency->PointInTriangle( Vi );

				if ( tri < 0 ) {
					assert( false );
					return false;
				}	

				// check whether the point lies exactly on one edge of the triangle
				int edgeIdx = adjacency->PointInTriangleEdge( Vi, tri );
				if ( edgeIdx >= 0 ) {
					// split the edge by Vi
					int result[ 4 ];
					adjacency->SplitEdge( edgeIdx, Vi, result );
					for( int j = 0; j < 4; j++ ) {
						if ( result[ j ] >= 0 ) {
							toCheck.addUnique( result[ j ] );
						}
					}
				} else {
					// split the triangle in 3
					int result[ 3 ];
					adjacency->SplitTriangle( tri, Vi, result );
					for( int j = 0; j < 3; j++ ) {
						toCheck.addUnique( result[ j ] );
					}
				}

				while( !toCheck.empty() ) {
					int tri = toCheck[ toCheck.size() - 1 ];
					toCheck.resize( toCheck.size() - 1 , false );

					Adjacency<T>::Triangle_t& triangle = adjacency->triangles[ tri ];
					if ( !triangle.valid ) {
						continue;
					}

					// check delaunay condition
					for( int e = 0; e < 3; e++ ) {
						int adjacentIdx = adjacency->AdjacentTriangle( tri, e );
						if ( adjacentIdx < 0 ) {
							continue;
						}
						int globalEdgeIndex = abs( triangle.edges[ e ] ) - 1;
						const Adjacency<T>::Triangle_t& adjacent = adjacency->triangles[ adjacentIdx ];
						if ( !adjacent.valid ) {
							continue;
						}
						assert( adjacent.valid );
						int edgeFromAdjacent = adjacent.LocalEdgeIndex( globalEdgeIndex );
						assert( edgeFromAdjacent >= 0 );
						const int v = adjacency->VertexOutOfTriEdge( adjacentIdx, edgeFromAdjacent );
						assert( v >= 0 );
						assert( !triangle.Contains( v ) );

						if ( triangle.InsideCircumcircle( adjacency->vertices[ v ], adjacency->vertices ) > Delaunay2D::INSIDE_CIRCUMCIRCLE_EPSILON ) {
							int result[2];
							if ( adjacency->FlipTriangles( tri, adjacentIdx, result ) ) {
								toCheck.addUnique( result[0] );
								toCheck.addUnique( result[1] );
								break;
							}
						}
					}
				}
			}
			return true;
		}

		
		template< typename T >
		bool ConstrainedDelaunay2D( const LIST( RenderLib::Math::Vector2<T> )& vertices, 
									const LIST( int )& edges, 
									LIST(int)* outTriangles = NULL, 
									Adjacency<T>* adjacencyInfo = NULL ) {
			Adjacency<T>* adjacency = adjacencyInfo;
			if ( adjacencyInfo == NULL ) {
				adjacency = new Adjacency<T>;
			}

			if ( !Delaunay2D( vertices, NULL, adjacency ) ) {
				return false;
			}

			// build a list of all adjacent triangles for each vertex
			if ( !adjacency->BuildVertexTriangleAdjacency() ) {
				return false;
			}

			// insert edges
			for( size_t i = 0; i < edges.size(); i += 2 ) {
				int v0 = edges[ i ];
				int v1 = edges[ i + 1 ];
				if ( v0 == v1 || v0 < 0 || v1 < 0 ) {
					continue;
				}
				if ( adjacency->FindEdge( v0, v1 ) >= 0 ) {
					continue;
				}

#if _DEBUG && DEBUG_STEPS
				// dump to maya
				{				
					MPointArray vertices;
					for( int i = 0; i < adjacency->vertices.Num(); i++ ) {
						MPoint v( adjacency->vertices[ i ].x, adjacency->vertices[ i ].y, 0 );
						vertices.append( v );
					}

					MIntArray triangleVertices;
					for( int i = 0; i < adjacency->triangles.Num(); i++ ) {
						const Adjacency<T>::Triangle_t& triangle = adjacency->triangles[ i ];
						if ( !triangle.valid ) {
							continue;
						}
						triangleVertices.append( triangle.vertices[ 0 ] );
						triangleVertices.append( triangle.vertices[ 1 ] );
						triangleVertices.append( triangle.vertices[ 2 ] );
					}


					const int resultingTris = triangleVertices.length() / 3;
					MFnMeshData meshData;
					MObject meshObj = meshData.create();
					MFnMesh mesh( meshObj );
					MIntArray polygonCounts( resultingTris, 3 );
					MStatus status;
					mesh.create( vertices.length(), resultingTris, vertices, polygonCounts, triangleVertices, MObject::kNullObj, &status );
					MGlobal::addToModel( meshObj );			
				}
#endif
				LIST( int ) upperVertices, lowerVertices;
				if ( adjacency->DeleteTrianglesIntersectingSegment_r( v0, v1, upperVertices, lowerVertices ) == 0 ) {
					break;
				}

				if ( !upperVertices.empty() && !adjacency->TriangulateSubPolygon_r( upperVertices ) ) {
					break;
				}
				if ( !lowerVertices.empty() && !adjacency->TriangulateSubPolygon_r( lowerVertices ) ) {
					break;
				}
			}
		
#if _DEBUG && 0
			// dump to maya
			{				
				MPointArray vertices;
				for( int i = 0; i < adjacency->vertices.Num(); i++ ) {
					MPoint v( adjacency->vertices[ i ].x, adjacency->vertices[ i ].y, 0 );
					vertices.append( v );
				}

				MIntArray triangleVertices;
				for( int i = 0; i < adjacency->triangles.Num(); i++ ) {
					const Adjacency<T>::Triangle_t& triangle = adjacency->triangles[ i ];
					if ( !triangle.valid ) {
						continue;
					}
					triangleVertices.append( triangle.vertices[ 0 ] );
					triangleVertices.append( triangle.vertices[ 1 ] );
					triangleVertices.append( triangle.vertices[ 2 ] );
				}


				const int resultingTris = triangleVertices.length() / 3;
				MFnMeshData meshData;
				MObject meshObj = meshData.create();
				MFnMesh mesh( meshObj );
				MIntArray polygonCounts( resultingTris, 3 );
				MStatus status;
				mesh.create( vertices.length(), resultingTris, vertices, polygonCounts, triangleVertices, MObject::kNullObj, &status );
				MGlobal::addToModel( meshObj );			
			}
#endif
			if ( adjacencyInfo == NULL ) {
				delete adjacency;
			}

			return true;
		}
		
		bool IsTriOutside( RenderLib::Math::Vector2f centroid, 
						   const float diagonal, 
						   const LIST( int ) &edges, 
						   const LIST( RenderLib::Math::Vector2f )& vertices, 
						   int triIdx );

} // namespace Delaunay2D
} // namespace Geometry	
} // namespace RenderLib



