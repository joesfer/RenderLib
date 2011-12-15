#pragma once
#include <math/algebra/point/point3.h>

namespace RenderLib {
namespace DataStructures {

/*
	===============================================================================

		PhotonMapKNN

		Intermediate class used to gather and evaluate the 
		nearest-neighbors queries

	===============================================================================
	*/
	class PhotonMapKNN {
	public:
		PhotonMapKNN( int _maxSamples, float _searchRadius ) : 
			maxSamples( _maxSamples ), 
			sqMaxSearchRadius( _searchRadius * _searchRadius ), 
			found(0), heapBuilt(false), squaredDist(NULL), index(NULL) {} 

		void BuildMaxHeap();
		void AddSample( SampleIndex_t s, float squaredDist);

		// search delimiters
		const int	maxSamples; 
		const float	sqMaxSearchRadius;	// squared max search radius

		int			found;				// samples actually found in the search radius (may be < maxSamples)
		bool		heapBuilt;			// At the end of the search, the squaredDist array will be arranged as a max heap. This flag will tell us when this is done.

		RenderLib::Math::Point3f	pos;
		float*		squaredDist;		//
		SampleIndex_t*	index;			// Index inside of the tree array. It's equivalent to a pointer inside the kd-tree
	private:
		// mark as uncopyable
		PhotonMapKNN(const PhotonMapKNN&);
		PhotonMapKNN& operator=(const PhotonMapKNN&);
	};
}
}