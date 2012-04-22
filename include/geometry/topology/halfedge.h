#pragma once
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
#include <math/algebra/vector/vector2.h>
#include <math/algebra/vector/vector3.h>
#include <math/algebra/point/point3.h>
#include <vector>

namespace RenderLib {
namespace Geometry {

//////////////////////////////////////////////////////////////////////////
// The HalfEdgeMesh class implements a half-edge mesh representation
// for topological queries and adjacency information retrieval.
//
// The implementation on this class is based on the description provided
// in http://fgiesen.wordpress.com/2012/02/21/half-edge-based-mesh-representations-theory/
// Additional information and resources can be found at:
// http://www.flipcode.com/archives/The_Half-Edge_Data_Structure.shtml
// http://www.cs.sunysb.edu/~gu/.../lecture_8_halfedge_data_structure.pdf
//
// The class does not assume triangular faces, although the constructors
// may do for the input geometry.
//////////////////////////////////////////////////////////////////////////
class HalfEdgeMesh
{
public:

	bool buildFromTriangles(const std::vector<int>& indices);

	typedef int edgeIndex_t;
	typedef int vertexIndex_t;
	typedef int faceIndex_t;

	/*
	   \                       /
	    \                     /     e = halfEdge
	     \                   /|     p = prevEdge(e)
	      \|                /       n = nextEdge(e)
	       b------->---o---a        o = oppositeEdge(e)
	       b-----<--e-----a         a = startVertex(e)
	      /                \        b = endVertex(e)
	    |/                  p       F = face(e)
	    /          F        /\
	   n                      \
	*/

	static const edgeIndex_t INVALID_EDGE = -1;
	static const faceIndex_t INVALID_FACE = -1;
	static const vertexIndex_t INVALID_VERTEX = -1;

	// return all faces sharing an edge with f
	void adjacentFaces(faceIndex_t f, std::vector<faceIndex_t>& adjacent ) const;

private:
	// Next edge belonging to the same face
	edgeIndex_t		nextEdge( edgeIndex_t e ) const;
	// Previous edge belonging to the same face
	edgeIndex_t		prevEdge( edgeIndex_t e ) const;
	// Get the half-edge linking the same vertices in opposite direction, belonging to an adjacent face
	edgeIndex_t		oppositeEdge( edgeIndex_t e ) const;
	// Get face which the half-edge belongs to
	faceIndex_t		face( edgeIndex_t e ) const;
	// Get vertex from which the half-edge starts
	vertexIndex_t	startVertex( edgeIndex_t e ) const;
	// Get vertex where half-edge finishes
	vertexIndex_t	endVertex( edgeIndex_t e ) const;
	// Get a link to one of the outgoing half-edges
	edgeIndex_t		edgeFromVertex( vertexIndex_t v ) const;
	// Get a link to one of the half-edges that make up the face
	edgeIndex_t		edgeFromFace( faceIndex_t f ) const;

	bool checkConsistency() const;

private:

	struct Edge {
		HalfEdgeMesh::edgeIndex_t prev;
		HalfEdgeMesh::edgeIndex_t next;
		HalfEdgeMesh::edgeIndex_t opposite;
		HalfEdgeMesh::faceIndex_t face;
		HalfEdgeMesh::vertexIndex_t start;
	};
	std::vector<Edge> edges;
	std::vector<edgeIndex_t> faceToEdge;
	std::vector<edgeIndex_t> vertexToEdge;
};

} // namespace Geometry
} // namespace RenderLib