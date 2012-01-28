#include "precompiled.h"
#pragma hdrstop

#include <assert.h>
#include "kdTree.h"
#include <CoreLib.h>

namespace RenderLib {
namespace DataStructures {

	#define ENABLE_STATS 0
	#define ADAPTIVE_HEURISTIC 1

	KdTreeAllocator KdTree::memoryPool;

	//////////////////////////////////////////////////////////////////////////

	KdTreeNode_t::~KdTreeNode_t() {
	}

	//////////////////////////////////////////////////////////////////////////

	#if ENABLE_STATS
	int				           KdTree::treeDepth;
	int				           KdTree::numNodes;
	CoreLib::idList<KdTreeNode_t*>* KdTree::leaves;
	int				           KdTree::hitcount;
	CoreLib::idList<int>*	           KdTree::depths;
	//sdLock statsLock;
	#endif

	const float KdTree::costTraverse   = 0.3f;
	const float KdTree::costIntersect  = 1.0f;
	const float KdTree::costEmptyBonus = 1.0f;
	const unsigned int KdTree::heuristicSwitchThreshold = 4096;
	int			 KdTree::maxDepth = 30;
	int			 KdTree::maxTrisPerLeaf = 16;

	KdTree::KdTree() {
		root = NULL;
		KdTree::memoryPool.Init();
	}

	KdTree::~KdTree() {
		//delete( root ); // using a memory arena
		KdTree::memoryPool.FreeAll();
	#if ENABLE_STATS
		delete(KdTree::leaves);
		delete(KdTree::depths);
		KdTree::leaves = NULL;
		KdTree::depths = NULL;
	#endif
	}

	KdTreeNode_t* KdTree::CreateNode() {
		return memoryPool.CreateNode();
	}
	KdTreeNode_t* KdTree::AllocChildren() {
		return memoryPool.AllocChildren();
	}

	#if ENABLE_STATS
	#define MARK_AS_LEAF( nTriangles )\
		statsLock.Acquire();\
		KdTree::leaves->Append( node );\
		KdTree::depths->Append( depth );\
		statsLock.Release();\
		return;
	#else
	#define MARK_AS_LEAF( nTriangles )\
		return;
	#endif

	void KdTree::Build_r( KdTreeNode_t* node, const TriangleBounds_t* triangleBounds, int depth, const RenderLib::Geometry::BoundingBox& bounds ) {
		using namespace RenderLib::Geometry;
	#if ENABLE_STATS
		statsLock.Acquire();
		KdTree::treeDepth = max( treeDepth, depth );
		statsLock.Release();
	#endif
		
		if ( node->triangles.Num() <= KdTree::maxTrisPerLeaf || depth == KdTree::maxDepth ) {
			MARK_AS_LEAF( node->triangles.size() )
		}

		int bestSharedTris = 0;
		int bestPlaneType = 0;
		float bestSplitter = 0.f;

		float noSplitCost = costIntersect * node->triangles.Num(); // cost of not splitting the node
		float bestSplitCost = FLT_MAX;

		CoreLib::idList< int > bestLeftTriangles;
		CoreLib::idList< int > bestRightTriangles;
		bestLeftTriangles.AssureSize( node->triangles.Num() / 2 );
		bestRightTriangles.AssureSize( node->triangles.Num() / 2 );
		bestLeftTriangles.SetGranularity( 128 );
		bestRightTriangles.SetGranularity( 128 );

		assert( node->triangles.Num() > 0 );

		float cost;
		for ( int j = 0; j < 3; j++ ) {
			int planeType = j;
			int sharedTris = 0;
	#if ADAPTIVE_HEURISTIC		
			float splitter = depth > maxDepth / 4 || node->triangles.Num() < KdTree::heuristicSwitchThreshold ? 
								// more expensive and more accurate when we have fewer triangles per node
								FindSplitterSAH( triangleBounds, node->triangles, planeType, bounds, cost ) :
								// faster more general heuristic
								FindSplitterMedian( triangleBounds, node->triangles, planeType, bounds, cost );
	#else
			float splitter = FindSplitterSAH( triangleBounds, node->triangles, planeType, bounds, cost );
	#endif

			if ( j == 0  || cost < bestSplitCost ) {

				bestSplitter = splitter;
				bestPlaneType = planeType;
				bestSharedTris = sharedTris;
				bestSplitCost = cost;

				bestLeftTriangles.SetNum( 0, false );
				bestRightTriangles.SetNum( 0, false );

				BoundingBox leftBounds = bounds;
				leftBounds.max()[ bestPlaneType ] = bestSplitter;
				BoundingBox rightBounds = bounds;
				rightBounds.min()[ bestPlaneType ] = bestSplitter;

				for ( int i = 0; i < node->triangles.Num(); i++ ) {
					int tri = node->triangles[ i ];
					const BoundingBox& triBounds = triangleBounds[ tri ].bounds;

					if ( triBounds.overlaps( leftBounds ) ) { 
						bestLeftTriangles.Append( tri );
					}
					if ( triBounds.overlaps( rightBounds ) ) { 
						bestRightTriangles.Append( tri );
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

		node->children = AllocChildren();
		node->planeType = bestPlaneType;
		node->splitPlanePos = bestSplitter;

		node->children[ 0 ].triangles.Swap( bestLeftTriangles );
		node->children[ 1 ].triangles.Swap( bestRightTriangles );
		node->triangles.Clear();
		
	#if ENABLE_STATS
		statsLock.Acquire();
		KdTree::numNodes += 2;
		statsLock.Release();
	#endif

		Build_r( &node->children[ 0 ], triangleBounds, depth + 1, leftBounds );
		Build_r( &node->children[ 1 ], triangleBounds, depth + 1, rightBounds );
	}

	float KdTree::FindSplitterSAH( const TriangleBounds_t* triangleBounds, const CoreLib::idList< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost ) {
		using namespace RenderLib::Geometry;

		// use surface area heuristic

		CoreLib::idList< float > sorter;
		sorter.SetNum( triangleIndices.Num() * 2 );
		
		for ( int i = 0; i < triangleIndices.Num(); i ++ ) {
			sorter[ 2 * i     ] = triangleBounds[ triangleIndices[ i ] ].bounds.min()[ axis ];
			sorter[ 2 * i + 1 ] = triangleBounds[ triangleIndices[ i ] ].bounds.max()[ axis ];	
		}
		sorter.Sort( SplitterSort );

		float bestCost = FLT_MAX;
		CoreLib::idList<float>::Iterator bestSplit = sorter.End();

		CoreLib::idList<int> left;
		CoreLib::idList<int> right = triangleIndices;

		CoreLib::idList< float > ::Iterator newStart = sorter.Begin();
		float furtherSplitPos = nodeBounds.max()[ axis ];
		while( newStart != sorter.End() && *newStart <= nodeBounds.min()[ axis ] ) { // find the first split pos inside the bounds
			newStart++;
		} 
			
		CoreLib::idList< float > ::Iterator splitIt = newStart;
		float split;
		while( splitIt != sorter.End() && *splitIt < furtherSplitPos ) {
			split = *splitIt;

			int leftCount = triangleIndices.Num() - right.Num();
			for ( int j = 0; j < right.Num(); j++ ) {			
				float minBound = triangleBounds[ right[ j ] ].bounds.min()[ axis ];
				float maxBound = triangleBounds[ right[ j ] ].bounds.max()[ axis ];

				if (   minBound < split || 
					 ( minBound == split && triangleBounds[ right[ j ] ].bounds.extents()[ axis ] < 1.0e-3f ) ) {
					leftCount++; // shared between left and right (both sides of the splitter)
				} 

				if (  maxBound < split || 
					( maxBound == split && triangleBounds[ right[ j ] ].bounds.extents()[ axis ] > 1.0e-3f ) ) { 
						// surely not on the right of the splitter
						right.RemoveIndexFast( j );
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
			float emptyBonus = ( ( leftCount == 0 && !leftDegenerate ) || ( right.Num() == 0 && !rightDegenerate ) ) ? costEmptyBonus : 0.0f;
			float cost = costTraverse + costIntersect * (1.0f - emptyBonus) * (probabilityLeft * leftCount + probabilityRight * right.Num());

			if ( cost < bestCost ) {
				bestCost = cost;
				bestSplit = splitIt;
			}

			// skip potentially duplicate split positions
			CoreLib::idList< float >::Iterator it2 = splitIt;
			do {
				it2++;
			} while ( it2 != sorter.End() && fabsf(*it2 - split ) < 1e-5f );	
			splitIt = it2;
		}

		splitCost = bestCost;
		return bestSplit != sorter.End() ? *bestSplit : FLT_MAX;
	}

	float KdTree::FindSplitterMedian( const TriangleBounds_t* triangleBounds, const CoreLib::idList< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost ) {

		// use median

		CoreLib::idList< float > sorter;
		sorter.SetNum( triangleIndices.Num() );
		for ( int i = 0; i < triangleIndices.Num(); i++ ) {
			sorter[ i ] = triangleBounds[ triangleIndices[ i ] ].centroid[ axis ];
		}

		sorter.Sort( SplitterSort );
		if ( ( sorter.Num() % 2 ) == 0 ) {
			int low = ( sorter.Num() - 1 ) / 2;
			int high = ( sorter.Num() ) / 2;
			return ( sorter[ low ] + sorter[ high ] ) * 0.5f;
		}

		int mid = ( sorter.Num() - 1 ) / 2;
		return sorter[ mid ];
	}

	int KdTree::SplitterSort( const float* a, const float* b ) {
		const float d = *a - *b;
		return d < 0.f ? -1 : ( d > 0.f ? 1 : 0 );
	}

	bool ClipSegment(float min, float max, float a, float b, float d, float& t0, float& t1) {
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

		t0 = __max(u0, t0);
		t1 = __min(u1, t1);

		if (t1 < t0) {
			return false;
		}

		return true; 
	}

	/*
	 clips segment A-B with bounding box defined by Min-Max
	 returns the clipping as two distances t0,t1 along the axis A-B
	*/
	bool ClipSegment(const RenderLib::Math::Point3f& A, const RenderLib::Math::Point3f& B, const RenderLib::Math::Point3f& Min, const RenderLib::Math::Point3f& Max, float& t0, float &t1 ) {
		using namespace RenderLib::Math;
		Vector3f D = (B - A);
		D.normalize();

		if (!ClipSegment(Min.x, Max.x, A.x, B.x, D.x, t0, t1) ||
			!ClipSegment(Min.y, Max.y, A.y, B.y, D.y, t0, t1) ||
			!ClipSegment(Min.z, Max.z, A.z, B.z, D.z, t0, t1)) {
			return false;
		}

		return true;
	}

	#if ENABLE_STATS

	void KdTree::Depth( int& min, int& max, int& avg ) const {
		min = INT_MAX;
		max = 0;
		int total = 0;
		for ( int i = 0; i < KdTree::depths->Num(); i++ ) {
			min = (*KdTree::depths)[ i ] < min ? (*KdTree::depths)[ i ] : min;
			max = (*KdTree::depths)[ i ] > max ? (*KdTree::depths)[ i ] : max;
			total += (*KdTree::depths)[ i ];
		}
		avg = total / KdTree::depths->Num();
	}


	void KdTree::LeavesStats( int& numLeaves, int& avgTrisPerLeaf, float& percentEmptyLeaves ) {
		numLeaves = KdTree::leaves->Num();
		avgTrisPerLeaf = 0;
		percentEmptyLeaves = numLeaves;
		for ( int i = 0; i < numLeaves; i++ ) {
			if ( (*KdTree::leaves)[ i ]->triangles.Num() > 0 ) {
				avgTrisPerLeaf += (*KdTree::leaves)[ i ]->triangles.Num(); 
				percentEmptyLeaves --;
			}
		}

		avgTrisPerLeaf /= numLeaves;
		percentEmptyLeaves /= numLeaves;

	}

	#endif

	//////////////////////////////////////////////////////////////////////////

	KdTreeAllocator::KdTreeAllocator() :
		chunkSize( 2048 * sizeof(KdTreeNode_t) ) { 
	}
	KdTreeAllocator::~KdTreeAllocator() {
		FreeAll();
	}

	void KdTreeAllocator::Init() { 
		firstChunk = new memChunk_t( chunkSize );
		currentChunk = firstChunk;
	}

	void KdTreeAllocator::FreeAll() {
		delete( firstChunk );
		firstChunk = NULL;
	}

	KdTreeNode_t* KdTreeAllocator::CreateNode() {
		if ( currentChunk->allocated + 1 > chunkSize / sizeof(KdTreeNode_t) ) {
			AllocChunk();
		}
		KdTreeNode_t* children = currentChunk->memory + currentChunk->allocated;
		memset( children, 0, sizeof( KdTreeNode_t ) );
		children->triangles.SetGranularity( 1024 );
		children->triangles.Clear();
		currentChunk->allocated ++;

		return children;
	}
	KdTreeNode_t* KdTreeAllocator::AllocChildren() {
		if ( currentChunk->allocated + 2 > chunkSize / sizeof(KdTreeNode_t) ) {
			AllocChunk();
		}
		KdTreeNode_t* children = currentChunk->memory + currentChunk->allocated;
		memset( children, 0, 2 * sizeof( KdTreeNode_t ) );
		children[ 0 ].triangles.SetGranularity( 1024 );
		children[ 0 ].triangles.Clear();
		children[ 1 ].triangles.SetGranularity( 1024 );
		children[ 1 ].triangles.Clear();
		currentChunk->allocated += 2;

		return children;
	}

	void KdTreeAllocator::AllocChunk( ) {
		currentChunk->nextChunk = new memChunk_t( chunkSize );
		currentChunk = currentChunk->nextChunk;
	};

}
}