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
namespace Delaunay {

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
		bool isValid() const;
		bool containsVertex( const int vert ) const;
		void getFaceVertices( const int f, int& a, int& b, int& c ) const;
	};

	//////////////////////////////////////////////////////////////////////////
	// class Delaunay3D
	//
	// 3D Delaunay tessellation (tetrahedralization)
	//////////////////////////////////////////////////////////////////////////
	
	class Delaunay3D {
	public:
#if LEAVE_CONTAINING_TETRAHEDRON 
		bool tetrahedralize( CoreLib::List< Point >& points, 
							 CoreLib::List< tetrahedron_t >& tetrahedra );
#else
		bool tetrahedralize( const CoreLib::List< Point >& points, 
							 CoreLib::List< tetrahedron_t >& tetrahedra );
#endif

	private:	

		CoreLib::List< REAL > tempVertices;

	};

} // namespace Delaunay
} // namespace Geometry
} // namespace RenderLib
