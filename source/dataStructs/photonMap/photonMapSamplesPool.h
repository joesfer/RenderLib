#pragma once

#include <malloc.h>

namespace RenderLib {
namespace DataStructs {

	/*
	===============================================================================

		PhotonMapSamplesPool

		Array of contiguous memory used to store the Photon Map samples. 

	===============================================================================
	*/
	template <class T>
	class PhotonMapSamplesPool {
	public:
		PhotonMapSamplesPool() : maxAllocated(0), reserved(0), memoryUsed(0), memoryPool(NULL) {}
		~PhotonMapSamplesPool() { freeMemory(); }

		bool allocate( unsigned int maxSamples );
		T* reserveChunk( unsigned int samples );
		T* releaseData( unsigned int& size );
		unsigned int dataSize() const { return reserved; }
		void condense();
		void merge( const PhotonMapSamplesPool<T>& other );

		PhotonMapSamplesPool<T>& operator = ( const PhotonMapSamplesPool<T>& other );
	
	private:
		void freeMemory();
	
	private:
		unsigned int	maxAllocated;	// in photons
		unsigned int	reserved;		// in photons

		unsigned int	memoryUsed;		// in bytes
		T*				memoryPool;
	};

	//////////////////////////////////////////////////////////////////////////
	
	/*
	===================
	PhotonMapSamplesPool::allocate
	===================
	*/
	template <class T>
	bool PhotonMapSamplesPool<T>::allocate( unsigned int maxSamples ) {
		maxAllocated = 0;
		reserved = 0; 
		memoryUsed = 0;
		try {
			// we will need an extra element in the array when we later organize the data as a kdtree
			// If we don't store it here, we would need a realloc later, and that could have a peak
			// memory usage of 2N if there is not enough contiguous space which would probably lead
			// into a crash (we're tight of space here, and the arrays are very large). 
			memoryUsed = (maxSamples + 1) * sizeof( T );
			memoryPool = (T*)calloc( (maxSamples + 1), sizeof( T ) ); 
			maxAllocated = maxSamples;
		} catch( char * ) {
			return false;
		}
		return true;
	}

	/*
	===================
	PhotonMapSamplesPool::addSample
	===================
	*/
	template <class T>
	T* PhotonMapSamplesPool<T>::reserveChunk( unsigned int samples ) {
		T* ptr = NULL;
		//lock.Acquire();
		if ( reserved + samples <= maxAllocated ) {
			ptr = memoryPool + reserved;
			reserved += samples;
		}
		//lock.Release();
		return ptr;
	}

	/*
	===================
	PhotonMapSamplesPool::releaseData

	Gives up the ownership of the stored memory and cleans the object
	We use this to avoid copying the samples on the PhotonMap
	===================
	*/
	template <class T>
	T* PhotonMapSamplesPool<T>::releaseData( unsigned int& size ) {
		//lock.Acquire();	
		T* data = memoryPool;
		memoryPool = NULL;
		size = maxAllocated; // note that the array has actually size+1 elements (see sdAOMemoryPool::Allocate)
		maxAllocated = 0;
		reserved = 0;
		memoryUsed = 0;
		//lock.Release();
		return data;
	}

	/*
	===================
	PhotonMapSamplesPool::condense
	===================
	*/
	template <class T>
	void PhotonMapSamplesPool<T>::condense() {
		if ( reserved == maxAllocated ) {
			return;
		}
		T* temp = memoryPool;
		memoryPool = (T*)malloc( reserved * sizeof(T) );
		memcpy( memoryPool, temp, reserved * sizeof(T) );
		free( temp );
		maxAllocated = reserved;
	}

	/*
	===================
	PhotonMapSamplesPool::merge
	===================
	*/
	template <class T>
	void PhotonMapSamplesPool<T>::merge( const PhotonMapSamplesPool<T>& other ) {

		if ( other.reserved == 0 ) { 
			return; 
		}

		T* temp = memoryPool;

		memoryUsed = (reserved + other.reserved + 1) * sizeof(T); // remember we're carrying an extra photon for the later kd-tree balance
		memoryPool = (T*)malloc( memoryUsed );
		memcpy( memoryPool, temp, reserved * sizeof(T) );
		memcpy( &memoryPool[ reserved ], other.memoryPool, other.reserved * sizeof(T) );
		free( temp );
		reserved += other.reserved;
		maxAllocated = reserved;
	}

	/*
	===================
	PhotonMapSamplesPool::merge
	===================
	*/
	template <class T>
	PhotonMapSamplesPool<T>& PhotonMapSamplesPool<T>::operator = ( const PhotonMapSamplesPool<T>& other ) {
		free(memoryPool);
		memoryPool = (T*)malloc( other.maxAllocated * sizeof(T) );
		reserved = other.reserved;
		maxAllocated = other.maxAllocated;
		return *this;
	}

	/*
	===================
	PhotonMapSamplesPool::freeMemory
	===================
	*/
	template <class T>
	void PhotonMapSamplesPool<T>::FreeMemory() {
		//lock.Acquire();
		if ( memoryPool != NULL ) {
			free( memoryPool );
			memoryPool = NULL;
			reserved = 0;
			maxAllocated = 0;
		}
		//lock.Release();
	}

}
}