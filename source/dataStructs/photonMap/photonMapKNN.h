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