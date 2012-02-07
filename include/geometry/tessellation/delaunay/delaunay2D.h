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

#include <coreLib.h>
#include <assert.h>
#include <stack>
#include <math/algebra/point/point2.h>
#include <math/algebra/vector/vector2.h>
#include <math/algebra/vector/vector3.h>
#include <math/algebra/matrix/matrix3.h>
#include <math/algebra/matrix/matrix4.h>
#include <math/constants.h>
#include <geometry/intersection/intersection.h>
#define DEBUG_STEPS 0

namespace RenderLib {
namespace Geometry {
namespace Delaunay {

	#define LIST( type ) CoreLib::List< type, CoreLib::Memory::StaticMemoryPool< type > >
	
	bool isTriOutside( RenderLib::Math::Vector2f centroid, 
					const float diagonal, 
					const LIST( int ) &edges, 
					const LIST( RenderLib::Math::Vector2f )& vertices, 
					int triIdx );



	////////////////////////////////////////////////////////////////////////////
	// AdjacencyInfo:
	// Auxiliary class containing topology information, used to store
	// information between calls to Delaunay2D and ConstrainedDelaunay2D
	////////////////////////////////////////////////////////////////////////////
	class AdjacencyInfo {
	public:
		AdjacencyInfo();

		int createTriangle( unsigned int a, unsigned int b, unsigned int c );
		void removeTriangle( unsigned int t );
		bool flipTriangles( int tri1, int tri2, int result[2] );

		// returns the triangle index of the triangle containing p
		int pointInTriangle( const RenderLib::Math::Vector2f& p ) const;
		int pointInTriangleEdge( const RenderLib::Math::Vector2f& p, const int t ) const;

		void splitEdge( int edgeIndex, const RenderLib::Math::Vector2f& p, int result[ 4 ] );
		void splitTriangle( int triIdx, const RenderLib::Math::Vector2f& p, int result[ 3 ] );


		int adjacentTriangle( int triIndex, int edgeIndex ) const;	
		int vertexOutOfTriEdge( int triIndex, int edgeIndex ) const;		
		bool worthFlipping( int A, int B, int C, int D ) const;
		float closestAngleOnTriangle( int vA, int vB, int vC ) const;

		bool buildVertexTriangleAdjacencyInfo();

		unsigned int deleteTrianglesIntersectingSegment_r( const int vIdx0, const int vIdx1, 
															LIST( int )& upperVertices,
															LIST( int )& lowerVertices );

		bool triangulateSubPolygon_r( const LIST( int )& indices );

		int findEdge( int v1, int v2 ) const;

		float segmentVertexSide( const RenderLib::Math::Vector2f& A, 
								 const RenderLib::Math::Vector2f& B, 
								 const RenderLib::Math::Vector2f& P ) const;

	public:
		struct Edge_t {
			int vertices[2]; // endpoints
			int triangles[2]; // triangles on the positive side [0], and negative side [1]

			Edge_t();

			bool Links( int v1, int v2 ) const;
		};
		struct Triangle_t {
			int vertices[3];
			int edges[3]; // signed, 1-based
			bool valid;
			RenderLib::Math::Point2f	cirumcircleCenter;
			float							circumcircleRadius;

			Triangle_t();
			
			bool contains( int v ) const;

			bool contains( int a, int b, int c ) const;

			int localEdgeIndex( int globalEdgeIndex ) const;			

			float insideCircumcircle( const RenderLib::Math::Vector2f& p, const LIST( RenderLib::Math::Vector2f )& verts ) const;
		};			

	public:

		LIST( Edge_t )				edges;
		LIST( Triangle_t )			triangles;
		LIST( RenderLib::Math::Vector2f ) vertices;
		LIST( unsigned int )		invalidTriangles;
		
		typedef LIST( int ) triList_t;
		LIST( triList_t )	vertexTriangleAdjacencyInfo;

	//private:
		unsigned int createEdge( int v1, int v2 );

		int commonEdge( const Triangle_t& tri1, const Triangle_t& tri2 ) const;

	};

		

	/////////////////////////////////////////////////////////////////////////////////////
	// Delaunay2D:
	// Implements Lawson algorithm: 
	// 	http://www.henrikzimmer.com/VoronoiDelaunay.pdf
	// 	
	// 	Unlike the Watson algorithm the Lawson algorithm is not based finding circumcircles and deleting
	// 	triangles to obtain an insertion polygon. Lawson's incremental insertion algorithm utilizes edge
	// 	flippings to achieve the same result and thus avoids a possibly faulty, degenerate mesh which can
	// 	occur using Watsons method.
	// 	In the same way as before, if we are not given a start triangulation we use a super triangle
	// 	enclosing all vertices in V . In every insertion step a vertex p belonging to V is inserted. 
	// 	A simple retriangulation is made where the edges are inserted between p and the corner vertices of 
	// 	the triangle containing p (fig.11). For all new triangles formed the circumcircles are checked, if they contain
	// 	any neighbouring vertex the edge between them is flipped. This process is executed recursively
	// 	until there are no more faulty triangles and no more flips are required (fig.12). The algorithm then
	// 	moves on, inserting another vertex from V until they have all been triangulated into a mesh, then,
	// 	like before the super triangle and its edge are removed.
	/////////////////////////////////////////////////////////////////////////////////////
	bool delaunay2D( const LIST( RenderLib::Math::Vector2f )& vertices, 
					 LIST(int)* outTriangles = NULL, 
					 AdjacencyInfo* adjacencyInfo = NULL );

	bool constrainedDelaunay2D( const LIST( RenderLib::Math::Vector2f )& vertices, 
								const LIST( int )& edges, 
								LIST(int)* outTriangles = NULL, 
								AdjacencyInfo* adjacencyInfo = NULL );
		
} // namespace Delaunay
} // namespace Geometry	
} // namespace RenderLib



