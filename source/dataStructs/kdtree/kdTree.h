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

		void init();
		void freeAll();

		KdTreeNode_t* createNode();
		KdTreeNode_t* allocChildren();
	private:	

		void allocChunk();

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
		bool 					init( const RenderLib::DataStructures::ITriangleSoup< T >* mesh, int maxDepth, int minTrisPerLeaf );

		void 					release();
		
		template< typename T >
		bool traceClosest( const TraceDesc& trace, const RenderLib::DataStructures::ITriangleSoup< T >* mesh, TraceIsectDesc& isect ) const;

		RenderLib::Geometry::BoundingBox	bounds() const { return boundingBox; }

	private:
		static void build_r( KdTreeNode_t* node, const TriangleBounds_t* triangleBounds, int depth, const RenderLib::Geometry::BoundingBox& bounds );
		
		static float findSplitterSAH( const TriangleBounds_t* triangleBounds, const CoreLib::idList< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost );
		static float findSplitterMedian( const TriangleBounds_t* triangleBounds, const CoreLib::idList< int >& triangleIndices, int axis, const RenderLib::Geometry::BoundingBox& nodeBounds, float& splitCost );
		
		static int splitterSort( const float* a, const float* b );
		static KdTreeNode_t* createNode();
		static KdTreeNode_t* allocChildren();

	private:
		KdTreeNode_t*						root;
		RenderLib::Geometry::BoundingBox	boundingBox;

		static const float costTraverse;
		static const float costIntersect;
		static const float costEmptyBonus;
		static		 int maxSimultaneousThreads;
		static	const unsigned int heuristicSwitchThreshold;

		static int maxDepth;
		static int maxTrisPerLeaf;

		static KdTreeAllocator memoryPool;
	};

	bool clipSegment(const RenderLib::Math::Point3f& A, const RenderLib::Math::Point3f& B, const RenderLib::Math::Point3f& Min, const RenderLib::Math::Point3f& Max, float& t0, float &t1 );

	#include "kdTree.inl"
}
}
