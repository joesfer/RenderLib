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
#include <geometry/topology/halfedge.h>
namespace RenderLib {
namespace Geometry {

bool HalfEdgeMesh::buildFromTriangles(const std::vector<int>& indices) {
	using namespace std;

	int numVertices = 0;
	vector<int>::const_iterator idxIt = indices.cbegin();
	while( idxIt != indices.cend() ) {
		numVertices = std::max(numVertices, *idxIt + 1);
		idxIt++;
	}
	
	faceToEdge.resize( indices.size() / 3 );
	vertexToEdge.resize( numVertices );
		
	edges.clear();
	edges.reserve( indices.size() );
	
	faceIndex_t face;
	Edge faceEdges[3];	

	for( size_t i = 0; i < indices.size(); i += 3 ) {
		face = faceIndex_t( i / 3 );
		for( int j = 0; j < 3; j++ ) {
			Edge& e = faceEdges[j];
			e.face = face;
			e.start = vertexIndex_t(indices[i + j]);
			e.prev = edgeIndex_t(edges.size() + (j + 2) % 3);
			e.next = edgeIndex_t(edges.size() + (j + 1) % 3);
			e.opposite = INVALID_EDGE;
			vertexToEdge[ e.start ] = edgeIndex_t(edges.size() + j);
		}

		for( int j = 0; j < 3; j++ ) edges.push_back(faceEdges[j]);
		faceToEdge[ face ] = edges.size() - 1;
	}

	for( edgeIndex_t e1 = 0; e1 < edgeIndex_t(edges.size()); e1++ ) {
		Edge& edge1 = edges[e1];
		for( edgeIndex_t e2 = e1 + 1; e2 < edgeIndex_t(edges.size()); e2++ ) {			
			Edge& edge2 = edges[e2];
			if ( edge2.opposite != INVALID_EDGE ) continue;
			if ( endVertex(e2) == edge1.start &&
				 startVertex(e2) == endVertex(e1)) {
				edge2.opposite = e1;
				edge1.opposite = e2;
				break;
			}
		}
	}

	return checkConsistency();
}

bool HalfEdgeMesh::checkConsistency() const {
	/* Half-edge invariants
	For every half-edge e, e = prev(next(e)) = next(prev(e)) = opposite(opposite(e)), and face(e) = face(next(e)).
	For every vertex v, start(vertEdge(v)) = v.
	For every face f, face(faceEdge(f)) = f.
	*/
	for( edgeIndex_t e = 0; e < edgeIndex_t(edges.size()); e++ ) {
		const Edge& edge = edges[e];
		if ( e != edges[edge.prev].next ) return false;
		if ( e != edges[edge.next].prev ) return false;
		if ( edge.opposite != INVALID_EDGE && e != edges[edge.opposite].opposite ) return false;
		if ( edge.face != edges[edge.next].face ) return false;
	}
	for( vertexIndex_t v = 0; v < vertexIndex_t(vertexToEdge.size()); v++ ) {
		if ( edges[ vertexToEdge[v] ].start != v ) return false;
	}
	for( faceIndex_t f = 0; f < faceIndex_t(faceToEdge.size()); f++ ) {
		if ( edges[ faceToEdge[f] ].face != f ) return false;
	}
	return true;

}

HalfEdgeMesh::edgeIndex_t		HalfEdgeMesh::nextEdge( edgeIndex_t e ) const {
	if( e >= 0 && (size_t)e < edges.size() )
		return edges[e].next;
	else
		return INVALID_EDGE;
}
HalfEdgeMesh::edgeIndex_t		HalfEdgeMesh::prevEdge( edgeIndex_t e ) const {
	if( e >= 0 && (size_t)e < edges.size() )
		return edges[e].prev;
	else
		return INVALID_EDGE;
}
HalfEdgeMesh::edgeIndex_t HalfEdgeMesh::oppositeEdge( edgeIndex_t e ) const {
	if( e >= 0 && (size_t)e < edges.size() )
		return edges[e].opposite;
	else
		return INVALID_EDGE;
}
HalfEdgeMesh::faceIndex_t HalfEdgeMesh::face( edgeIndex_t e ) const {
	if( e >= 0 && (size_t)e < edges.size() )
		return edges[e].face;
	else
		return INVALID_FACE;
}
HalfEdgeMesh::vertexIndex_t HalfEdgeMesh::startVertex( edgeIndex_t e ) const {
	if( e >= 0 && (size_t)e < edges.size() )
		return edges[e].start;
	else
		return INVALID_VERTEX;
}
HalfEdgeMesh::vertexIndex_t HalfEdgeMesh::endVertex( edgeIndex_t e ) const {
	return startVertex(nextEdge(e));
}
HalfEdgeMesh::edgeIndex_t HalfEdgeMesh::edgeFromVertex( vertexIndex_t v ) const {
	if( v >= 0 && (size_t)v < vertexToEdge.size() )
		return vertexToEdge[v];
	else
		return INVALID_EDGE;
}
HalfEdgeMesh::edgeIndex_t HalfEdgeMesh::edgeFromFace( faceIndex_t f ) const {
	if( f >= 0 && (size_t)f < faceToEdge.size() )
		return faceToEdge[f];
	else
		return INVALID_EDGE;
}

void HalfEdgeMesh::adjacentFaces(faceIndex_t f, std::vector<faceIndex_t>& adjacent ) const {
	edgeIndex_t e = edgeFromFace(f);
	if ( e == INVALID_EDGE ) return;
	vertexIndex_t start = startVertex(e);
	do {
		faceIndex_t adjFace = face(oppositeEdge(e));
		if ( adjFace != INVALID_FACE ) adjacent.push_back(adjFace);
		e = nextEdge(e);
	} while( startVertex(e) != start );
}

} // namespace Geometry
} // namespace Renderlib