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

template< typename T >
bool KdTree::init( const RenderLib::DataStructures::ITriangleSoup<T>* mesh, const int _maxDepth, const int _minTrisPerLeaf ) {
	using namespace RenderLib::Math;

	KdTree::maxDepth       = _maxDepth;
	KdTree::maxTrisPerLeaf = _minTrisPerLeaf;

	delete( root );

	this->root = createNode();

	size_t numTris = mesh->numIndices() / 3;
	const int *indices = mesh->getIndices();

	this->root->triangles.resize( numTris, true );
	TriangleBounds_t* triangleBounds = new TriangleBounds_t[ numTris ];

	for ( size_t i = 0; i < numTris ; i++ ) {

		this->root->triangles[ i ] = (int)i;

		const Point3f &p0 = mesh->getVertices()[ indices[ 3 * i ]     ].position;
		const Point3f &p1 = mesh->getVertices()[ indices[ 3 * i + 1 ] ].position;
		const Point3f &p2 = mesh->getVertices()[ indices[ 3 * i + 2 ] ].position;

		triangleBounds[ i ].centroid = ( p0 + p1 + p2 ) * ( 1.f / 3.f );
		triangleBounds[ i ].bounds.expand( p0 );
		triangleBounds[ i ].bounds.expand( p1 );
		triangleBounds[ i ].bounds.expand( p2 );

		boundingBox.expand( triangleBounds[ i ].bounds );
	};

	// Grow the bounding box a tiny amount proportional to the scene dimensions for robustness.
	Vector3f offset = ( boundingBox.max() - boundingBox.min() ) * 1.0e-5f;
	boundingBox.min() -= offset;
	boundingBox.max() += offset;

	build_r( root, triangleBounds, 0, boundingBox );

	// free resources
	delete(triangleBounds);

	return true;
}

template< typename T >
bool KdTree::traceClosest( const TraceDesc& trace, const RenderLib::DataStructures::ITriangleSoup<T>* mesh, TraceIsectDesc& isect ) const {
	using namespace RenderLib::Raytracing;
	using namespace RenderLib::Math;
	using namespace RenderLib::Geometry;

	Ray ray;
	ray.origin = trace.startPoint;
	ray.direction = trace.endPoint - ray.origin; 
	ray.tMax = ray.direction.normalize();	
	ray.tMin = 0;

	float tMin = ray.tMin, tMax = ray.tMax; // entry/exit signed distance

	const KdTreeNode_t* currNode = root;
	int stackElement = 0;
	KdTreeStackElement_t* traversalStack = (KdTreeStackElement_t*)alloca( TRAVERSAL_MAXDEPTH * sizeof(KdTreeStackElement_t));
	if ( traversalStack == NULL ) {
		return false;
	}

	// Intersect ray with scene bounds, find the entry and exit signed distances
	if ( clipSegment( trace.startPoint, trace.endPoint, boundingBox.min(), boundingBox.max(), tMin, tMax ) == false ) {
		return false;
	}

	assert( tMin <= tMax );

	const KdTreeNode_t *firstChild, *secondChild;	
	isect.triangleIndex = -1;
	isect.t = FLT_MAX;

	while( isect.t >= tMin ) {

		if ( currNode->IsLeaf() == false ) {
#pragma region INTERMEDIATE_NODE
			const int splitAxis = currNode->planeType;
			float tSplitPlane;
			if ( ray.direction[ splitAxis ] != 0 ) {
				tSplitPlane = (currNode->splitPlanePos - ray.origin[splitAxis]) / ray.direction[ splitAxis ];
			} else { 
				tSplitPlane = FLT_MAX; // parallel ray, will never hit the plane
			}

			// Get the child nodes
			if ( ray.origin[splitAxis] <= currNode->splitPlanePos ) { // left child first
				firstChild = currNode->children;
				secondChild = firstChild + 1;
			} else { // right child first
				secondChild = currNode->children;
				firstChild = secondChild + 1;
			};

			// Proceed with the first child, potentially queuing the second one

			if (tSplitPlane > tMax || tSplitPlane < 0) {
				// no need to process secondChild
				currNode = firstChild;
			} else if (tSplitPlane < tMin) {
				// no need to process firstChild
				currNode = secondChild;
			} else {
				// We have to process both children. Start by the first one, and queue the second one in the stack                  
				traversalStack[ stackElement ].node = secondChild;
				traversalStack[ stackElement ].tMin = tSplitPlane;
				traversalStack[ stackElement ].tMax = tMax;
				stackElement++;

				currNode = firstChild;
				tMax = tSplitPlane;

				assert(stackElement < TRAVERSAL_MAXDEPTH);
			}			
#pragma endregion
		} else {

#pragma region LEAF_NODE
			// Check for intersection against the primitives in this node

			const int *indices       = mesh->getIndices();
			const T* verts = mesh->getVertices();
			Point3f start = ray.origin + ray.direction * tMin;
			Point3f end   = ray.origin + ray.direction * tMax;

#pragma region INTERSECTION_TEST
			float t, v, w;

			if ( trace.doubleSided == true ) {			
				for ( size_t i = 0; i < currNode->triangles.size(); i++ ) {

					const int triangleOffset = currNode->triangles[ i ] * 3;

					const Point3f& p0 = verts[ indices[ triangleOffset ]    ].position;
					const Point3f& p1 = verts[ indices[ triangleOffset + 1] ].position;
					const Point3f& p2 = verts[ indices[ triangleOffset + 2] ].position;

					if ( SegmentTriangleIntersect_DoubleSided( ray.origin, ray.direction, tMin, tMax, p0, p1, p2, t, v, w ) ) {
						if ( trace.testOnly ) {
							return true;
						}
						else if ( t < isect.t ) {
							isect.triangleIndex = currNode->triangles[ i ];
							assert( isect.triangleIndex >= 0 );
							isect.t = t;
							isect.v = v;
							isect.w = w;
						}
					}
				}
			} else {
				for ( size_t i = 0; i < currNode->triangles.size(); i++ ) {

					const int triangleOffset = currNode->triangles[ i ] * 3;

					const Point3f& p0 = verts[ indices[ triangleOffset ]    ].position;
					const Point3f& p1 = verts[ indices[ triangleOffset + 1] ].position;
					const Point3f& p2 = verts[ indices[ triangleOffset + 2] ].position;

					if ( SegmentTriangleIntersect_SingleSided( trace.startPoint, trace.endPoint, p0, p1, p2, t, v, w ) ) {
						if ( trace.testOnly ) {
							return true;
						} else if ( t < isect.t ) {
							isect.triangleIndex = currNode->triangles[ i ];
							assert( isect.triangleIndex >= 0 );
							isect.t = t;
							isect.v = v;
							isect.w = w;
						}
					}			
				}
			}
#pragma endregion
			
			// Pop the following node from the traversal stack
			if (stackElement > 0) {
				stackElement--;
				currNode = traversalStack[ stackElement ].node;
				tMin     = traversalStack[ stackElement ].tMin;
				tMax     = traversalStack[ stackElement ].tMax;
			} else {
				break; // done
			}
#pragma endregion
		}
	}

	return isect.triangleIndex >= 0;
}