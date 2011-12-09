#pragma once

#include <math/algebra/vector/vector3.h>
#include <math/algebra/point/point3.h>
#include "PhotonMapKNN.h"
#include "PhotonMapSamplesPool.h"
#include "geometry/Bounds/boundingbox.h"

#include <malloc.h>

namespace RenderLib {
namespace DataStructs {
	
	/*
	===============================================================================

		PhotonMap

		Tree structure for fast nearest-neighbors queries
	
	===============================================================================
	*/
	template <class T>
	class PhotonMap {
	public:
		
		PhotonMap( PhotonMapSamplesPool<T>& samples );
		~PhotonMap();
		
		void nearestSamples( const Point3f& pos, int maxSamples, float searchRadius, T** neighbors, int& found );

		// raw samples map
		unsigned int numSamples() const { return storedSamples; }
		const T*	 getSamples() const { return sampleMap; }
		T*			 getSamples() { return sampleMap; }

	private:
		void balanceKDTree(); // arrange the point cloud as a kd-tree for fast kNN queries. Call this once we're done adding new samples
		void balanceSubTree_r(T** srcArray, const int idx, const int srcIdx, const int finalIdx, T** balancedTree);
		void medianPartition(T** tree, const int begin, const int end, const int median, const int axis);
		void nearestSamples_r(PhotonMapKNN<T>* nearestSamples, unsigned int indexArray ) const;

		T* sampleMap;	// Organized as an array first, and then balanced as a binary tree where the i-th child is located at 2*i and its sibling at 2*i+1
		BoundingBox bbox;	// tree bbox

		unsigned int storedSamples;
		unsigned int halfStoredSamples;

	};

	//////////////////////////////////////////////////////////////////////////

	/*
	===================
	PhotonMap::PhotonMap
	===================
	*/
	template <class T>
	PhotonMap<T>::PhotonMap( PhotonMapSamplesPool<T>& samples ) {
		storedSamples = samples.getDataSize();
		halfStoredSamples = storedSamples / 2 - 1;
		sampleMap = samples.releaseData( storedSamples ); // get the propriety of the memory pool data, we'll be in charge of deleting it now.
		
		for ( unsigned int i = 0; i < storedSamples; i++ ) {
			bbox.AddPoint( sampleMap[ i ].GetPos() );
		}

		// note that we've already stored the extra element we need for the kdtree 
		// on photonMapSamplesPool::Allocate. This way we save the realloc to grow the array.
		balanceKDTree();
	}

	/*
	===================
	PhotonMap::~PhotonMap
	===================
	*/
	template <class T>
	PhotonMap<T>::~PhotonMap() {
		free( sampleMap );
	}

	/*
	===================
	PhotonMap::nearestSamples
	===================
	*/
	template <class T>
	void PhotonMap<T>::nearestSamples( const Point3f& pos, int maxSamples, float maxDist, 
									   T** neighbors, // out: array of "maxSamples" size of pointers to T
									   int& found			 // out: number of samples found
									  ) {
		if ( storedSamples == 0 ) {
			return;
		}

		PhotonMapKNN<T> ns( maxSamples, maxDist );

		memset( neighbors, 0, maxSamples * sizeof( T* ) );
		ns.squaredDist	= (float*)alloca( (maxSamples+1) * sizeof(float) );
		if ( ns.squaredDist == NULL ) {
			return;
		}
		ns.index		= neighbors;

		ns.pos					= pos;
		ns.found				= 0;
		ns.heapBuilt			= false;
		ns.squaredDist[0]		= maxDist * maxDist;

		// Fetch the nearest N samples, searching the whole tree (idx = 1 = root)
		nearestSamples_r( &ns, 1 );

		found = ns.found;
	}

	/*
	===================
	PhotonMap::nearestSamples_r
	===================
	*/
	template <class T>
	void PhotonMap<T>::nearestSamples_r( PhotonMapKNN<T>* nearestSamples, unsigned int arrayIndex ) const {
		T* sample = &sampleMap[arrayIndex];

		float distance;

		if ( arrayIndex < ( storedSamples >> 1 ) ) {
			// if we haven't surpassed half of the stored nodes, we're in an internal node of the tree
			// (beyond that limit, everything is a leaf in the array), so we search recursively

			// Split plane distance
			int splitPlane = sample->getSplitPlane();
			distance = nearestSamples->pos[splitPlane] - sample->getPos( splitPlane );

			if ( distance > 0.0f ) {
				// we're in the right-side leaf
				nearestSamples_r( nearestSamples, 2 * arrayIndex + 1 ); // 2 * arrayIndex + 1 = right child			
				if ( distance * distance < nearestSamples->squaredDist[0] ) {
					// we can still keep searching, the split plane is closer than the max search radius
					// search in the left node
					nearestSamples_r( nearestSamples, 2 * arrayIndex );
				}
			} else {
				// we're in the left-side leaf
				nearestSamples_r( nearestSamples, 2 * arrayIndex ); //  2 * arrayIndex = left child
				if ( distance * distance < nearestSamples->squaredDist[0] ) {
					// we can still keep searching, the split plane is closer than the max search radius
					// search in the right node
					nearestSamples_r( nearestSamples, 2 * arrayIndex + 1 );
				}
			}
		}

		// if we haven't gone through the last "if", we're in a leaf node

		if ( !sample->isValid() ) { 
			return;
		}

		// Calculate the squared distance from the requested position and the sample
		float dist2 = ( sample->getPos() - nearestSamples->pos ).lengthSquared( );
		if ( dist2 < nearestSamples->squaredDist[0] ) {
			// sample is inside the search radius
			if ( nearestSamples->found < nearestSamples->maxSamples ) {
				// we can keep this sample	
				nearestSamples->found++;
				nearestSamples->squaredDist[ nearestSamples->found ] = dist2;
				nearestSamples->index[ nearestSamples->found ] = sample;
			} else {
				// The array is full, so we must use the heap to see if we accept or reject this new sample 
				if ( nearestSamples->heapBuilt == false ) {
					nearestSamples->buildMaxHeap();
				}

				// Add the new sample into the max heap 
				nearestSamples->addSample( sample, dist2 );
			}
		}
	}


	/*
	===================
	PhotonMap::balanceKDTree
	===================
	*/
	template <class T>
	void PhotonMap<T>::balanceKDTree() {

		// FIXME: the reason why we use +1 element is to operate with indices 1..N, instead of 0..N-1
		// and simplify the median calculation and tree balancing. But this can be done better and save
		// that extra element.

		if ( storedSamples > 1) {
			// temp arrays for sorting
			T **balancedAux = (T**)malloc((storedSamples+1)*sizeof(T*));
			T **aux			= (T**)malloc((storedSamples+1)*sizeof(T*));

			//sampleMap[ storedSamples ] = sampleMap[ storedSamples - 1 ];
			for ( unsigned int i = 0; i <= storedSamples; i++) { 
				aux[ i ] = &sampleMap[ i ];
			}

			balanceSubTree_r( aux, 1, 1, storedSamples, balancedAux );
			free(aux);

			// Rearrange the balanced KDTree, building a heap
			unsigned int offset, j = 1;
			unsigned auxIdx = 1;

			T auxSample = sampleMap[j];

			for( unsigned int i = 1; i <= storedSamples; i++ ) {
				offset = (int)( balancedAux[j] - sampleMap ); 
				balancedAux[j] = NULL;
				if (offset != auxIdx) {
					sampleMap[j] = sampleMap[offset];
				} else {
					sampleMap[j] = auxSample;

					if (i < storedSamples) {
						for ( ; auxIdx <= storedSamples; auxIdx++) { 
							if(balancedAux[auxIdx] != NULL) { 
								break;
							}
						}
						auxSample = sampleMap[auxIdx];
						j = auxIdx;
					}
					continue;
				}
				j = offset;
			}
			free(balancedAux);
		}

		halfStoredSamples = storedSamples / 2 - 1;
	}

	#define SWAP_SAMPLES(samples,a,b) {\
		T* aux = samples[a];\
		samples[a] = samples[b];\
		samples[b]=aux;\
	}
	 	
	/*
	===================
	PhotonMap::MedianPartition
	===================
	*/
	template <class T>
	void PhotonMap<T>::medianPartition( T **tree, const int begin, const int end, const int median, const int axis )
	{
		int left = begin;
		int right = end;

		while(right > left) {
			const float v = tree[right]->getPos()[ axis ];
			int i = left - 1;
			int j = right;
			for ( ; ; )
			{
				while(tree[++i]->getPos()[ axis ] < v) {}; // i++
				while((tree[--j]->getPos()[ axis ] > v) && (j>left)) {}; // j--
				if ( i >= j ) {
					break;
				}
				SWAP_SAMPLES(tree,i,j);
			};
			SWAP_SAMPLES(tree, i, right);
			if (i >= median) {
				right = i - 1;
			}
			if (i <= median) {
				left = i + 1;
			}
		}
	}
	
	/*
	===================
	PhotonMap::balanceSubTree_r
	===================
	*/
	template <class T>
	void PhotonMap<T>::balanceSubTree_r(T** srcArray, const int index, const int startIndex, const int endIndex, T **balancedTree) {
		// Calculate the new median
		
		int median = 1;
		while( ( median * 4 ) <= ( endIndex - startIndex + 1) )	{
			median += median;
		}

		if ( ( median * 3 ) <= ( endIndex - startIndex + 1) ) {
			median += median;
			median += startIndex - 1;
		} else {
			median = endIndex - median + 1;
		}

		// determine the best split axis

		int axis = 2;
		Vector3f size = bbox.GetSize();
		if (size[0] > size[1] &&  size[0] > size[2]) { 
			axis = 0;
		} else {
			if ( size[1] > size[2] ) {
				axis = 1;
			}
		}

		// Divide the tree according to the chosen axis

		medianPartition(srcArray, startIndex, endIndex, median, axis);

		balancedTree[index] = srcArray[median];
		balancedTree[index]->setSplitPlane( axis );

		// Balance left and right nodes recursively

		if ( median > startIndex ) {
			// Balance left subtree
			if ( startIndex < median - 1 ) {
				const float auxBounds = bbox.GetMaxs()[ axis ];
				bbox.max()[ axis ] = balancedTree[ index ]->GetPos()[ axis ]; 
				BalanceSubTree_r(srcArray, 2 * index, startIndex, median - 1, balancedTree ); 
				bbox.max()[ axis ] = auxBounds;
			} else {
				balancedTree[ 2 * index ] = srcArray[ startIndex ];
			}
		}

		if ( median < endIndex ) {
			// Balance right subtree
			if (median + 1 < endIndex) {
				const float auxBounds = bbox.GetMins()[ axis ];
				bbox.min()[ axis ] = balancedTree[ index ]->GetPos()[ axis ]; 
				BalanceSubTree_r( srcArray, 2 * index + 1, median + 1, endIndex, balancedTree ); 
				bbox.min()[ axis ] = auxBounds;		
			} else {
				balancedTree[ 2 * index + 1 ] = srcArray[ endIndex ];
			}
		}
	}

}
}