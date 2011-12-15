#pragma once

#include <math/algebra/point/point3.h>
#include <geometry/bounds/boundingBox.h>

#include <vector>

namespace RenderLib {
namespace DataStructures {
		
	class PhotonMapKNN;

	typedef unsigned int SampleIndex_t;
	struct PhotonMapSample_t {
		RenderLib::Math::Point3f position;
		short		split;
	};

	/*
	===============================================================================

		PhotonMap

		Transforms an array of Point3d samples into a sorted flattened tree 
		structure for fast nearest-neighbors query.

	===============================================================================
	*/
	class PhotonMap {
	public:
		
		PhotonMap( const ::std::vector<RenderLib::Math::Point3f>& samples );
		~PhotonMap();

		void nearestSamples( const ::RenderLib::Math::Point3f& pos,	// in: center of the lookup query
							 int maxSamples,						// in: maximum number of neighbors to fetch
							 float searchRadius,					// in: maximum distance to neighbors
							 SampleIndex_t* neighbors,				// in/out: user-allocated array of maxSamples elements where results will be stored.
							 int& found								// out: number of nearest neighbors found
							 );
	
	private:
		void balanceKDTree(); // arrange the point cloud as a kd-tree for fast kNN queries. Call this once we're done adding new samples
		void balanceSubTree_r(PhotonMapSample_t** srcArray, const int idx, const int srcIdx, const int finalIdx, PhotonMapSample_t** balancedTree);
		void medianPartition(PhotonMapSample_t** tree, const int begin, const int end, const int median, const int axis);
		void nearestSamples_r(PhotonMapKNN* nearestSamples, SampleIndex_t indexArray ) const;

		::std::vector<PhotonMapSample_t>		sampleMap;	// Organized as an array first, and then balanced as a binary tree 
														// where the i-th child is located at 2*i and its sibling at 2*i+1		
		::std::vector<size_t>					mapping;	// Transforms internal indices back to the original order, so that 
														// ::nearestSamples index results match the provided samples array.
		RenderLib::Geometry::BoundingBox	bbox;		// tree bounds
	};
}
}