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

		tetrahedron_t();

		tetrahedron_t( const tetrahedron_t& other );

		void markInvalid();
		
		bool isValid() const;
		
		bool containsVertex( const int vert ) const;

		void getFaceVertices( const int f, int& a, int& b, int& c ) const;

		REAL getFaceArea( const int f, const CoreLib::List< Point >& vertices ) const;

		int getFaceFromVertices( const int a, const int b, const int c ) const;

		int getVertexOutsideFace( int f ) const;
	

		// We use this function to check whether two faces sharing the same vertices are reversed
		// without needing to calculate the normal
		bool sameWinding( const int v1[3], const int v2[3] ) const;

		int sharedFace( const tetrahedron_t& other, bool reversed ) const;

		bool adjacentTo( const tetrahedron_t& other ) const;

		bool checkNeighbors( const int thisIndex, const CoreLib::List< tetrahedron_t >& tetrahedra, const CoreLib::List< Point >& vertices ) const;

		void destroy( CoreLib::List< tetrahedron_t >& tetrahedra );

		// checks whether 'face' has the same orientation than 'otherFace' (normals point toward the same direction)
		bool sameOrientation( const int face, const tetrahedron_t& other, const int otherFace, const CoreLib::List< Point >& vertices ) const;

		void reverseFace( const int f );
		
		//////////////////////////////////////////////////////////////////////////
		// tetrahedron_t::IsFlat
		//
		// A tetrahedron is flat if its 4 vertices lie in the same plane
		// A flat tetrahedron has no circumsphere
		//////////////////////////////////////////////////////////////////////////
		bool isFlat( const CoreLib::List< Point >& vertices ) const;

		// ensure the tetrahedron centroid lies behind every face
		void fixFaceOrientations( const CoreLib::List< Point >& vertices );
		
		bool checkFaceOrientations( const CoreLib::List< Point >& vertices ) const;	
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
		bool tetrahedralize( CoreLib::List< Point >& points, 
							 CoreLib::List< tetrahedron_t >& tetrahedra );
#else
		bool tetrahedralize( const CoreLib::List< Point >& points, 
							 CoreLib::List< tetrahedron_t >& tetrahedra );
#endif

	private:
	
		// Predicates //////////////////////////////////////////////////////////////////////////

		// Determines whether p is above the plane ( > 0 ), below ( < 0 ) or on the plane ( = 0 )
		static REAL orient(  const Point& a, 
							 const Point& b, 
							 const Point& c, 
							 const Point& p );

		// returns > 0 if p is inside the sphere described by A,B,C,D, < 0 outside, = 0 on the sphere
		static REAL inSphere( const Point& a, 
							  const Point& b, 
							  const Point& c, 
							  const Point& d, 
							  const Point& p );

		// Is p inside tetrahedron t?
		static bool inside( const Point& p, const tetrahedron_t& t, const CoreLib::List< Point >& vertices );

		// whether 4 points are coplanar
		static bool coplanar( const Point& a, const Point& b, const Point& c, const Point& d );

		// Bistellar flips /////////////////////////////////////////////////////////////////////

		static void flip14( const unsigned int pointIndex, const unsigned int tetrahedron, 
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[4] );

		static bool flip23( const unsigned int tetrahedron1, const unsigned int tetrahedron2, 							
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[3] );

		static void flip32( const unsigned int tetrahedron1, const unsigned int tetrahedron2, const unsigned int tetrahedron3, 							
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[2] );

		static void flip44( const unsigned int tetrahedron1, const unsigned int tetrahedron2, const unsigned int tetrahedron3, const unsigned int tetrahedron4,							
							CoreLib::List< tetrahedron_t >& tetrahedra,
							const CoreLib::List< Point >& vertices,
							unsigned int resultingTetrahedra[4] );

		// Auxiliary operations ////////////////////////////////////////////////////////////////

		static void containingTetrahedron( const RenderLib::Math::Point3f& center, const float radius, // circumsphere  
											tetrahedron_t& t, CoreLib::List< Point >& points );

		// Given a tetrahedron and a face, ensures the potential adjacent tetrahedron sharing
		// that face points to the given tetrahedron.
		static void adjustNeighborVicinity( const int iT, const int f, CoreLib::List< tetrahedron_t >& tetrahedra );

		// returns the index of the tetrahedron containing p, or -1 if not found. 
		// It retrieves the result by searching from a given sourceT tetrahedron index. 
		static int walk( const Point& p, const int sourceT, const CoreLib::List< Point >& vertices, const CoreLib::List< tetrahedron_t >& tetrahedra );

		// Flips two given tetrahedra: T and Ta, which are non-Delaunay, 
		// into a Delaunay configuration using bistellar flips
		static void flip( const int T, const int Ta, const int p,
						const CoreLib::List< Point >& vertices,
						CoreLib::List< tetrahedron_t >& tetrahedra,
						std::stack< unsigned int >& needTesting );

		static void insertOnePoint( const CoreLib::List< Point >& vertices, const int pointIndex, 
									CoreLib::List< tetrahedron_t >& tetrahedra );

	private:

		CoreLib::List< REAL > tempVertices;

	};

} // namespace Delaunay3D
} // namespace Geometry
} // namespace RenderLib
