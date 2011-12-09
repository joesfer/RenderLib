#pragma once

namespace RenderLib {
namespace DataStructs {

	/*
	===============================================================================

		PhotonMapKNN

		Intermediate class used to gather and evaluate the 
		nearest-neighbors queries

	===============================================================================
	*/
	template <class T>
	class PhotonMapKNN {
	public:
		PhotonMapKNN( int _maxSamples, float _searchRadius ) : 
			maxSamples( _maxSamples ), 
			sqMaxSearchRadius( _searchRadius * _searchRadius ), 
			found(0), heapBuilt(false), squaredDist(NULL), index(NULL) {} 

		void buildMaxHeap();
		void addSample( T* s, float squaredDist);

	public:
		// search delimiters
		const int	maxSamples; 
		const float	sqMaxSearchRadius;	// squared max search radius

		int			found;				// samples actually found in the search radius (may be < maxSamples)
		bool		heapBuilt;			// At the end of the search, the squaredDist array will be arranged as a max heap. This flag will tell us when this is done.

		Point3f		pos;
		Vector3f	normal;
		float*		squaredDist;		//
		T**			index;				// Index inside of the tree array. It's equivalent to a pointer inside the kd-tree

	};
		
	//////////////////////////////////////////////////////////////////////////

	/*
	===================
	PhotonMapKNN::buildMaxHeap
	===================
	*/
	template <class T>
	void PhotonMapKNN<T>::buildMaxHeap() {
		float dst2;
		T* sample;
		int halfFound = found >> 1;
		int j, parent;

		for (int k = halfFound; k >= 1; k--) {
			parent = k;
			sample = index[k];
			dst2 = squaredDist[k];
			while(parent <= halfFound) {
				j = parent+parent; // Left child
				if ((j < found) && (squaredDist[j] < squaredDist[j+1])) {
					j++;
				}
				if (dst2 >= squaredDist[j]) {
					break;
				}
				squaredDist[parent] = squaredDist[j];
				index[parent] = index[j];
				parent = j;
			}
			squaredDist[parent] = dst2;
			index[parent] = sample;
		}
		heapBuilt = true;
	}

	/*
	===================
	PhotonMapKNN::addSample
	===================
	*/
	template <class T>
	void PhotonMapKNN<T>::addSample( T* s, float sqDist)
	{
		// Add the new sample into the max heap: remove the largest element, insert the new one and rearrange the heap
		
		int parent = 1, j = 2;
		while( j <= found) {
			if ( ( j < found ) && ( squaredDist[j] < squaredDist[j+1] ) ) {
				j++;
			}
			if ( sqDist > squaredDist[j]) {
				break;
			}
			squaredDist[parent] = squaredDist[j];
			index[parent] = index[j];
			parent = j;
			j += j;
		}
		index[parent] = s;
		squaredDist[parent] = sqDist;
		squaredDist[0] = squaredDist[1];
	}
}
}