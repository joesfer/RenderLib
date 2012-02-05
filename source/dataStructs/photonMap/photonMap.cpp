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

#include <dataStructs/photonMap/photonMap.h>
#include <dataStructs/photonMap/photonMapKNN.h>
#include <memory.h>

namespace RenderLib {
namespace DataStructures {
		
	//////////////////////////////////////////////////////////////////////////
	// PhotonMap
	//////////////////////////////////////////////////////////////////////////

	/*
	===================
	PhotonMap::PhotonMap
	===================
	*/

	PhotonMap::PhotonMap( const std::vector<RenderLib::Math::Point3f>& samples ) {
		
		sampleMap.resize(samples.size());
		for( size_t i = 0; i < samples.size(); i++ ) {
			sampleMap[i].position = samples[i];
		}
		
		for ( unsigned int i = 0; i < sampleMap.size(); i++ ) {
			bbox.expand( sampleMap[ i ].position );
		}

		balanceKDTree();
	}

	/*
	===================
	PhotonMap::~PhotonMap
	===================
	*/
	PhotonMap::~PhotonMap() {
	}

	/*
	===================
	PhotonMap::nearestSamples
	===================
	*/

	void PhotonMap::nearestSamples( const ::RenderLib::Math::Point3f& pos, int maxSamples, float maxDist, 
									SampleIndex_t* neighbors, // out: array of "maxSamples" size of pointers to T
									int& found			 // out: number of samples found
								   ) {
		if ( sampleMap.size() == 0 ) {
			return;
		}

		PhotonMapKNN ns( maxSamples, maxDist );

		memset( neighbors, 0, maxSamples * sizeof( SampleIndex_t ) );
		ns.squaredDist	= (float*)alloca( (maxSamples+1) * sizeof(float) );
		if ( ns.squaredDist == NULL ) {
			return;
		}
		ns.index		= neighbors;

		ns.pos					= pos;
		ns.found				= 0;
		ns.heapBuilt			= false;
		ns.squaredDist[0]		= maxDist * maxDist;

		// Fetch the nearest N samples, searching the whole tree (idx = 0 = root)
		nearestSamples_r( &ns, 0 );

		for( int i = 1; i <= ns.found; i++ ) {
			ns.index[i] = mapping[ns.index[i]];
		}
		found = ns.found;
	}

	/*
	===================
	PhotonMap::nearestSamples_r
	===================
	*/

	void PhotonMap::nearestSamples_r( PhotonMapKNN* nearestSamples, SampleIndex_t arrayIndex ) const {
		const PhotonMapSample_t& sample = sampleMap[arrayIndex];

		float distance;

		if ( arrayIndex < ( sampleMap.size() >> 1 ) - 1 ) {
			// if we haven't surpassed half of the stored nodes, we're in an internal node of the tree
			// (beyond that limit, everything is a leaf in the array), so we search recursively

			// Split plane distance
			int splitPlane = sample.split;
			distance = nearestSamples->pos[splitPlane] - sample.position[ splitPlane ];
			const int leftChild = 2 * ( arrayIndex + 1 ) - 1;
			const int rightChild = leftChild + 1;
			if (distance > 0.0f) {
				// we're in the right-side leaf
				nearestSamples_r( nearestSamples, rightChild ); // right child			
				if ( distance * distance < nearestSamples->squaredDist[0] ) {
					// we can still keep searching, the split plane is closer than the max search radius
					// search in the left node
					nearestSamples_r( nearestSamples, leftChild ); // left child
				};
			} else {
				// we're in the left-side leaf
				nearestSamples_r( nearestSamples, leftChild ); //  left child
				if ( distance * distance < nearestSamples->squaredDist[0] ) {
					// we can still keep searching, the split plane is closer than the max search radius
					// search in the right node
					nearestSamples_r( nearestSamples, rightChild );
				}
			}
		}

		// if we haven't gone through the last "if", we're in a leaf node

		// Calculate the squared distance from the requested position and the sample
		float dist2 = ( sample.position - nearestSamples->pos ).lengthSquared( );
		if ( dist2 < nearestSamples->squaredDist[0] ) {
			// sample is inside the search radius
			if ( nearestSamples->found < nearestSamples->maxSamples ) {
				// we can keep this sample	
				nearestSamples->found++;
				nearestSamples->squaredDist[ nearestSamples->found ] = dist2;
				nearestSamples->index[ nearestSamples->found ] = arrayIndex;
			} else {
				// The array is full, so we must use the heap to see if we accept or reject this new sample 
				if ( nearestSamples->heapBuilt == false ) {
					nearestSamples->BuildMaxHeap();
				}

				// Add the new sample into the max heap 
				nearestSamples->AddSample( arrayIndex, dist2 );
			}
		}
	}


	/*
	===================
	PhotonMap::balanceKDTree
	===================
	*/

	void PhotonMap::balanceKDTree() {
		
		if ( sampleMap.size() > 1) {
			// temp arrays for sorting
			using namespace std;
			vector<PhotonMapSample_t*> balancedAux;
			balancedAux.resize(sampleMap.size());
			{
				vector<PhotonMapSample_t*> aux;
				aux.resize(sampleMap.size());

				for ( size_t i = 0; i < sampleMap.size(); i++) { 
					aux[ i ] = &sampleMap[ i ];
				}

				balanceSubTree_r( &aux[0], 0, 0, sampleMap.size() - 1, &balancedAux[0] );
			}

			// Rearrange the balanced KDTree, building a heap
			int offset, j = 0;
			int auxIdx = 0;

			PhotonMapSample_t auxSample = sampleMap[j];
			mapping.resize( sampleMap.size() );
			for(unsigned int i = 0; i < sampleMap.size(); i++) {
				assert(balancedAux[j] != NULL);
				offset = (int)( balancedAux[j] - &sampleMap[0] ); 
				balancedAux[j] = NULL;
				if (offset != auxIdx) {
					sampleMap[j] = sampleMap[offset];
					mapping[j] = offset;
				} else {
					sampleMap[j] = auxSample;
					mapping[j] = auxIdx;

					if (i < sampleMap.size()) {
						for ( ; auxIdx < (int)sampleMap.size() - 1; auxIdx++) { 
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
		}
	}

	#define SWAP_SAMPLES(samples,a,b) {\
		PhotonMapSample_t* aux = samples[a];\
		samples[a] = samples[b];\
		samples[b]=aux;\
	}
	 	
	/*
	===================
	PhotonMap::medianPartition
	===================
	*/

	void PhotonMap::medianPartition( PhotonMapSample_t **tree, const int begin, const int end, const int median, const int axis )
	{
		int left = begin;
		int right = end;

		while(right > left) {
			const float v = tree[right]->position[ axis ];
			int i = left - 1;
			int j = right;
			for ( ; ; )
			{
				while(tree[++i]->position[ axis ] < v) {}; // i++
				while((tree[--j]->position[ axis ] > v) && (j>left)) {}; // j--
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
	
	int findmedian(int start, int end) {
		int median = 1;
		int count = end - start + 1;

		while( ( median * 4 ) <= count )	{
			median += median;
		}

		if ( ( median * 3 ) <= count ) {
			median += median;
			median += start - 1;
		} else {
			median = end - median + 1;
		}
		assert(median >= start && median <= end);
		return median;
	}

	/*
	===================
	PhotonMap::balanceSubTree_r
	===================
	*/
	void PhotonMap::balanceSubTree_r(PhotonMapSample_t** srcArray, const int index, const int startIndex, const int endIndex, PhotonMapSample_t** balancedTree) {
		// Calculate the new median
		
		int median = findmedian(startIndex + 1, endIndex + 1) - 1; // turn to 0-based again (was 1-based)

		// determine the best split axis

		int axis = bbox.longestAxis();

		// Divide the tree according to the chosen axis

		medianPartition(srcArray, startIndex, endIndex, median, axis);

		balancedTree[index] = srcArray[median];
		balancedTree[index]->split = (short)axis;

		// balance left and right nodes recursively

		const int leftChildren = 2 * (index + 1) - 1;
		const int rightChildren = leftChildren + 1;
		if ( median > startIndex ) {
			// balance left subtree
			if ( startIndex < median - 1 ) {
				const float auxBounds = bbox.max()[ axis ];
				bbox.max()[ axis ] = balancedTree[ index ]->position[ axis ]; 
				balanceSubTree_r(srcArray, leftChildren, startIndex, median - 1, balancedTree ); 
				bbox.max()[ axis ] = auxBounds;
			} else {
				balancedTree[ leftChildren ] = srcArray[ startIndex ];
			}
		}

		if ( median < endIndex ) {
			// balance right subtree
			if (median + 1 < endIndex) {
				const float auxBounds = bbox.min()[ axis ];
				bbox.min()[ axis ] = balancedTree[ index ]->position[ axis ]; 
				balanceSubTree_r( srcArray, rightChildren, median + 1, endIndex, balancedTree ); 
				bbox.min()[ axis ] = auxBounds;		
			} else {
				balancedTree[ rightChildren ] = srcArray[ endIndex ];
			}
		}
	}
		
	//////////////////////////////////////////////////////////////////////////
	// PhotonMapKNN
	//////////////////////////////////////////////////////////////////////////

	/*
	===================
	PhotonMapKNN::BuildMaxHeap
	===================
	*/
	void PhotonMapKNN::BuildMaxHeap() {
		float dst2;
		SampleIndex_t sample;
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
	PhotonMapKNN::AddSample
	===================
	*/
	void PhotonMapKNN::AddSample( SampleIndex_t s, float sqDist)
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