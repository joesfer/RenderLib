#pragma once
#include "math/algebra/point/point3.h"
#include "geometry/bounds/boundingBox.h"
#include "dataStructs/triangleSoup/triangleSoup.h"
#include "raytracing/ray/ray.h"
#include <malloc.h>
#include <coreLib.h>

namespace RenderLib {
namespace DataStructures {
	#define TRAVERSAL_MAXDEPTH 50 // TODO: this can be calculated with a formula given the max tree depth

	struct TraceIsectDesc {
		int triangleIndex;
		unsigned int indices[3];
		float t, v, w; // t = ray distance; v,w = barycentric coordinates
	};

	struct TraceDesc {
		RenderLib::Math::Point3f startPoint;
		RenderLib::Math::Point3f endPoint;
		bool doubleSided;
		bool testOnly;
	};

	//////////////////////////////////////////////////////////////////////////

	struct TriangleBounds_t {
		RenderLib::Geometry::BoundingBox	bounds;
		RenderLib::Math::Point3f		centroid;
	};

	//////////////////////////////////////////////////////////////////////////

	struct KdTreeNode_t {
		KdTreeNode_t( void ) : 
		children( NULL ) {};

		~KdTreeNode_t( void );

		inline bool						IsLeaf() const { return children == NULL; }

		int								planeType;
		float                           splitPlanePos;
		CoreLib::idList< int >			triangles;
		KdTreeNode_t*					children;
	};

	//////////////////////////////////////////////////////////////////////////

	class KdTreeAllocator {
	public:
		KdTreeAllocator();
		~KdTreeAllocator();

		void Init();
		void FreeAll();

		KdTreeNode_t* CreateNode();
		KdTreeNode_t* AllocChildren();
	private:	

		void AllocChunk();

		struct memChunk_t {
			memChunk_t( size_t bytes ) { memory = (KdTreeNode_t*)malloc( bytes ); allocated = 0; nextChunk = NULL; }
			KdTreeNode_t*  memory;
			size_t allocated;
			memChunk_t* nextChunk;
		};

		memChunk_t* firstChunk;
		memChunk_t* currentChunk;
		const size_t chunkSize;
	};

	//////////////////////////////////////////////////////////////////////////

	struct KdTreeStackElement_t {
		const KdTreeNode_t* node;  // pointer to far child
		float tMin, tMax;
	};

	//////////////////////////////////////////////////////////////////////////

	class KdTree {
	public:
								KdTree();
		virtual					~KdTree();

		template< typename T >
		bool 					Init( const RenderLib::DataStructures::ITriangleSoup< T >* mesh, int maxDepth, int minTrisPerLeaf );

		void 					Release();
		
		template< typename T >
		bool TraceClosest( const TraceDesc& trace, const RenderLib::DataStructures::ITriangleSoup< T >* mesh, TraceIsectDesc& isect ) const;

		RenderLib::Geometry::BoundingBox	Bounds() const { return bounds; }

	#if ENABLE_STATS
		int                     MaxDepth() const { return maxDepth; } // max depth allowed
		int                     NumNodes() const { return numNodes; }
		void                    Depth( int& min, int& max, int& avg ) const; // min depth, max depth, avg depth reached by the built tree
		int                     MaxTrisPerLeaf() const { return maxTrisPerLeaf; }
		void                    LeavesStats( int& numLeaves, int& avgTrisPerLeaf, float& percentEmptyLeaves );
	#endif

	private:
		static void Build_r( KdTreeNode_t* node, const TriangleBounds_t* triangleBounds, int depth, const RenderLib::Geometry::BoundingBox& bounds );
		
		static float FindSplitterSAH( const TriangleBounds_t* triangleBounds, const CoreLib::idList< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost );
		static float FindSplitterMedian( const TriangleBounds_t* triangleBounds, const CoreLib::idList< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost );
		
		static int SplitterSort( const float* a, const float* b );
		static KdTreeNode_t* CreateNode();
		static KdTreeNode_t* AllocChildren();

	private:
		KdTreeNode_t*			  root;
		RenderLib::Geometry::BoundingBox				  bounds;

		static const float costTraverse;
		static const float costIntersect;
		static const float costEmptyBonus;
		static		 int maxSimultaneousThreads;
		static	const unsigned int heuristicSwitchThreshold;

		static int maxDepth;
		static int maxTrisPerLeaf;

	#if ENABLE_STATS
		// statistics
		static int                     treeDepth;
		static int                     numNodes;
		static int                     hitcount;
		// we cannot leave idLists as static types because the stub won't be initialized 
		// if their constructor is called when Arkitekt's DLL is loaded. Therefore they're
		// set as pointers, and created afterwards.
		static CoreLib::idList<node_t*>*        leaves;
		static CoreLib::idList<int>*            depths;
	#endif
		static KdTreeAllocator memoryPool;
	};

	bool ClipSegment(const RenderLib::Math::Point3f& A, const RenderLib::Math::Point3f& B, const RenderLib::Math::Point3f& Min, const RenderLib::Math::Point3f& Max, float& t0, float &t1 );

	#include "kdTree.inl"
}
}
