#include "precompiled.h"
#pragma hdrstop

#include <assert.h>
#include "dataStructs/kdtree/kdTree.h"
#include <CoreLib.h>

namespace RenderLib {
namespace DataStructures {

	#define ADAPTIVE_HEURISTIC 1

	KdTreeAllocator KdTree::memoryPool;

	//////////////////////////////////////////////////////////////////////////

	KdTreeNode_t::~KdTreeNode_t() {
	}

	//////////////////////////////////////////////////////////////////////////	

	const float KdTree::costTraverse   = 0.3f;
	const float KdTree::costIntersect  = 1.0f;
	const float KdTree::costEmptyBonus = 1.0f;
	const unsigned int KdTree::heuristicSwitchThreshold = 4096;
	int			 KdTree::maxDepth = 30;
	int			 KdTree::maxTrisPerLeaf = 16;

	KdTree::KdTree() {
		root = NULL;
		KdTree::memoryPool.init();
	}

	KdTree::~KdTree() {
		//delete( root ); // using a memory arena
		KdTree::memoryPool.freeAll();	
	}

	KdTreeNode_t* KdTree::createNode() {
		return memoryPool.createNode();
	}
	KdTreeNode_t* KdTree::allocChildren() {
		return memoryPool.allocChildren();
	}
	
	#define MARK_AS_LEAF( nTriangles )\
		return;

	void KdTree::build_r( KdTreeNode_t* node, const TriangleBounds_t* triangleBounds, int depth, const RenderLib::Geometry::BoundingBox& bounds ) {
		using namespace RenderLib::Geometry;
		
		if ( node->triangles.size() <= (size_t)KdTree::maxTrisPerLeaf || depth == KdTree::maxDepth ) {
			MARK_AS_LEAF( node->triangles.size() )
		}

		int bestSharedTris = 0;
		int bestPlaneType = 0;
		float bestSplitter = 0.f;

		float noSplitCost = costIntersect * node->triangles.size(); // cost of not splitting the node
		float bestSplitCost = FLT_MAX;

		CoreLib::List< int > bestLeftTriangles;
		CoreLib::List< int > bestRightTriangles;
		bestLeftTriangles.preAllocate( node->triangles.size() / 2 );
		bestRightTriangles.preAllocate( node->triangles.size() / 2 );
		bestLeftTriangles.setGranularity( 128 );
		bestRightTriangles.setGranularity( 128 );

		assert( node->triangles.Num() > 0 );

		float cost;
		for ( int j = 0; j < 3; j++ ) {
			int planeType = j;
			int sharedTris = 0;
	#if ADAPTIVE_HEURISTIC		
			float splitter = depth > maxDepth / 4 || node->triangles.size() < (size_t)KdTree::heuristicSwitchThreshold ? 
								// more expensive and more accurate when we have fewer triangles per node
								findSplitterSAH( triangleBounds, node->triangles, planeType, bounds, cost ) :
								// faster more general heuristic
								findSplitterMedian( triangleBounds, node->triangles, planeType, bounds, cost );
	#else
			float splitter = findSplitterSAH( triangleBounds, node->triangles, planeType, bounds, cost );
	#endif

			if ( j == 0  || cost < bestSplitCost ) {

				bestSplitter = splitter;
				bestPlaneType = planeType;
				bestSharedTris = sharedTris;
				bestSplitCost = cost;

				bestLeftTriangles.resize( 0, false );
				bestRightTriangles.resize( 0, false );

				BoundingBox leftBounds = bounds;
				leftBounds.max()[ bestPlaneType ] = bestSplitter;
				BoundingBox rightBounds = bounds;
				rightBounds.min()[ bestPlaneType ] = bestSplitter;

				for ( size_t i = 0; i < node->triangles.size(); i++ ) {
					int tri = node->triangles[ i ];
					const BoundingBox& triBounds = triangleBounds[ tri ].bounds;

					if ( triBounds.overlaps( leftBounds ) ) { 
						bestLeftTriangles.append( tri );
					}
					if ( triBounds.overlaps( rightBounds ) ) { 
						bestRightTriangles.append( tri );
					}
				}

				assert( bestLeftTriangles.Num() + bestRightTriangles.Num() >= node->triangles.Num()) ;
			}
		}

		if ( bestSplitCost >= noSplitCost ) { // it is not worth splitting
			MARK_AS_LEAF( node->triangles.Num() )
		}

		BoundingBox leftBounds = bounds;
		leftBounds.max()[ bestPlaneType ] = bestSplitter;
		float leftCenter = ( leftBounds.max()[ bestPlaneType ] + leftBounds.min()[ bestPlaneType ] ) * 0.5f;
		float leftExtents = leftBounds.max()[ bestPlaneType ] - leftCenter;

		BoundingBox rightBounds = bounds;
		rightBounds.min()[ bestPlaneType ] = bestSplitter;
		float rightCenter = ( rightBounds.max()[ bestPlaneType ] + rightBounds.min()[ bestPlaneType ] ) * 0.5f;
		float rightExtents = rightBounds.max()[ bestPlaneType ] - rightCenter;

		if ( leftExtents < 1.0e-3f || rightExtents < 1.0e-3f ) {
			// the split is set in one of the planes of this node bounds,
			// producing one of the children with no volume and the other 
			// which is the same as the parent node. Avoid splitting.
			MARK_AS_LEAF( node->triangles.Num() )
		}

		node->children = allocChildren();
		node->planeType = bestPlaneType;
		node->splitPlanePos = bestSplitter;

		node->children[ 0 ].triangles.swap( bestLeftTriangles );
		node->children[ 1 ].triangles.swap( bestRightTriangles );
		node->triangles.clear();
		
		build_r( &node->children[ 0 ], triangleBounds, depth + 1, leftBounds );
		build_r( &node->children[ 1 ], triangleBounds, depth + 1, rightBounds );
	}

	float KdTree::findSplitterSAH( const TriangleBounds_t* triangleBounds, const CoreLib::List< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost ) {
		using namespace RenderLib::Geometry;

		// use surface area heuristic

		CoreLib::List< float > sorter;
		sorter.resize( triangleIndices.size() * 2 );
		
		for ( size_t i = 0; i < triangleIndices.size(); i ++ ) {
			sorter[ 2 * i     ] = triangleBounds[ triangleIndices[ i ] ].bounds.min()[ axis ];
			sorter[ 2 * i + 1 ] = triangleBounds[ triangleIndices[ i ] ].bounds.max()[ axis ];	
		}
		sorter.sort( splitterSort );

		float bestCost = FLT_MAX;
		CoreLib::List<float>::ConstIterator bestSplit = sorter.end();

		CoreLib::List<int> left;
		CoreLib::List<int> right = triangleIndices;

		CoreLib::List< float >::ConstIterator newStart = sorter.begin();
		float furtherSplitPos = nodeBounds.max()[ axis ];
		while( newStart != sorter.end() && *newStart <= nodeBounds.min()[ axis ] ) { // find the first split pos inside the bounds
			newStart++;
		} 
			
		CoreLib::List< float >::ConstIterator splitIt = newStart;
		float split;
		while( splitIt != sorter.end() && *splitIt < furtherSplitPos ) {
			split = *splitIt;

			size_t leftCount = triangleIndices.size() - right.size();
			for ( size_t j = 0; j < right.size(); j++ ) {			
				float minBound = triangleBounds[ right[ j ] ].bounds.min()[ axis ];
				float maxBound = triangleBounds[ right[ j ] ].bounds.max()[ axis ];

				if (   minBound < split || 
					 ( minBound == split && triangleBounds[ right[ j ] ].bounds.extents()[ axis ] < 1.0e-3f ) ) {
					leftCount++; // shared between left and right (both sides of the splitter)
				} 

				if (  maxBound < split || 
					( maxBound == split && triangleBounds[ right[ j ] ].bounds.extents()[ axis ] > 1.0e-3f ) ) { 
						// surely not on the right of the splitter
						right.removeIndexFast( j );
						j--;			
				}
			}		
			
			BoundingBox leftBounds = nodeBounds;
			leftBounds.max()[ axis ] = split;
			BoundingBox rightBounds = nodeBounds;
			rightBounds.min()[ axis ] = split;

			float probabilityLeft  = leftBounds.surfaceArea() / nodeBounds.surfaceArea();
			float probabilityRight =  rightBounds.surfaceArea() / nodeBounds.surfaceArea();
			const float boundsEpsilon = 0.1f;
			bool leftDegenerate  = leftBounds.extents().x < boundsEpsilon  || leftBounds.extents().y < boundsEpsilon  || leftBounds.extents().z < boundsEpsilon;
			bool rightDegenerate = rightBounds.extents().x < boundsEpsilon || rightBounds.extents().y < boundsEpsilon || rightBounds.extents().z < boundsEpsilon;
			float emptyBonus = ( ( leftCount == 0 && !leftDegenerate ) || ( right.size() == 0 && !rightDegenerate ) ) ? costEmptyBonus : 0.0f;
			float cost = costTraverse + costIntersect * (1.0f - emptyBonus) * (probabilityLeft * leftCount + probabilityRight * right.size());

			if ( cost < bestCost ) {
				bestCost = cost;
				bestSplit = splitIt;
			}

			// skip potentially duplicate split positions
			CoreLib::List< float >::ConstIterator it2 = splitIt;
			do {
				it2++;
			} while ( it2 != sorter.end() && fabsf(*it2 - split ) < 1e-5f );	
			splitIt = it2;
		}

		splitCost = bestCost;
		return bestSplit != sorter.end() ? *bestSplit : FLT_MAX;
	}

	float KdTree::findSplitterMedian( const TriangleBounds_t* triangleBounds, const CoreLib::List< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost ) {

		// use median

		CoreLib::List< float > sorter;
		sorter.resize( triangleIndices.size() );
		for ( size_t i = 0; i < triangleIndices.size(); i++ ) {
			sorter[ i ] = triangleBounds[ triangleIndices[ i ] ].centroid[ axis ];
		}

		sorter.sort( splitterSort );
		if ( ( sorter.size() % 2 ) == 0 ) {
			size_t low = ( sorter.size() - 1 ) / 2;
			size_t high = ( sorter.size() ) / 2;
			return ( sorter[ low ] + sorter[ high ] ) * 0.5f;
		}

		size_t mid = ( sorter.size() - 1 ) / 2;
		return sorter[ mid ];
	}

	int KdTree::splitterSort( const float* a, const float* b ) {
		const float d = *a - *b;
		return d < 0.f ? -1 : ( d > 0.f ? 1 : 0 );
	}

	bool clipSegment(float min, float max, float a, float b, float d, float& t0, float& t1) {
		const float threshold = 1.0e-6f;
		if ( fabsf(d) < threshold) {
			if (d > 0.0f) {
				return !(b < min || a > max);
			} else {
				return !(a < min || b > max);
			}
		}

		float u0, u1;

		u0 = (min - a) / d;
		u1 = (max - a) / d;

		if (u0 > u1) {
			float aux = u0; 
			u0 = u1; 
			u1 = aux;
		}

		if (u1 < t0 || u0 > t1) {
			return false;
		}

		t0 = std::max(u0, t0);
		t1 = std::min(u1, t1);

		if (t1 < t0) {
			return false;
		}

		return true; 
	}

	/*
	 clips segment A-B with bounding box defined by Min-Max
	 returns the clipping as two distances t0,t1 along the axis A-B
	*/
	bool clipSegment(const RenderLib::Math::Point3f& A, const RenderLib::Math::Point3f& B, const RenderLib::Math::Point3f& Min, const RenderLib::Math::Point3f& Max, float& t0, float &t1 ) {
		using namespace RenderLib::Math;
		Vector3f D = (B - A);
		D.normalize();

		if (!clipSegment(Min.x, Max.x, A.x, B.x, D.x, t0, t1) ||
			!clipSegment(Min.y, Max.y, A.y, B.y, D.y, t0, t1) ||
			!clipSegment(Min.z, Max.z, A.z, B.z, D.z, t0, t1)) {
			return false;
		}

		return true;
	}

	//////////////////////////////////////////////////////////////////////////

	KdTreeAllocator::KdTreeAllocator() :
		chunkSize( 2048 * sizeof(KdTreeNode_t) ) { 
	}
	KdTreeAllocator::~KdTreeAllocator() {
		freeAll();
	}

	void KdTreeAllocator::init() { 
		firstChunk = new memChunk_t( chunkSize );
		currentChunk = firstChunk;
	}

	void KdTreeAllocator::freeAll() {
		delete( firstChunk );
		firstChunk = NULL;
	}

	KdTreeNode_t* KdTreeAllocator::createNode() {
		if ( currentChunk->allocated + 1 > chunkSize / sizeof(KdTreeNode_t) ) {
			allocChunk();
		}
		KdTreeNode_t* children = currentChunk->memory + currentChunk->allocated;
		memset( children, 0, sizeof( KdTreeNode_t ) );
		children->triangles.setGranularity( 1024 );
		children->triangles.clear();
		currentChunk->allocated ++;

		return children;
	}
	KdTreeNode_t* KdTreeAllocator::allocChildren() {
		if ( currentChunk->allocated + 2 > chunkSize / sizeof(KdTreeNode_t) ) {
			allocChunk();
		}
		KdTreeNode_t* children = currentChunk->memory + currentChunk->allocated;
		memset( children, 0, 2 * sizeof( KdTreeNode_t ) );
		children[ 0 ].triangles.setGranularity( 1024 );
		children[ 0 ].triangles.clear();
		children[ 1 ].triangles.setGranularity( 1024 );
		children[ 1 ].triangles.clear();
		currentChunk->allocated += 2;

		return children;
	}

	void KdTreeAllocator::allocChunk( ) {
		currentChunk->nextChunk = new memChunk_t( chunkSize );
		currentChunk = currentChunk->nextChunk;
	};

}
}