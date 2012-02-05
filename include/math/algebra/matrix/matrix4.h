#pragma once

namespace RenderLib {
namespace Math {

	template< typename T >
	class Matrix4 {
	public:
		// Constructors ///
		Matrix4( void );
		Matrix4( const Matrix4<T>& other );
		Matrix4( T m00, T m01, T m02, T m03, 
				 T m10, T m11, T m12, T m13,
				 T m20, T m21, T m22, T m23,
				 T m30, T m31, T m32, T m33 );
		Matrix4( const T* m );

		// Operators ///

		Matrix4<T> transposed() const;
		Matrix4<T>& transpose();

		// Methods ///
		T determinant() const;

	public:
		T m[4][4];
	};

	#include "matrix4.inl"

	typedef Matrix4<float> Matrix4f;
	typedef Matrix4<double> Matrix4d;
}
}
