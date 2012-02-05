#pragma once

namespace RenderLib {
namespace Math {

	template< typename T >
	class Matrix3 {
	public:
		// Constructors ///
		Matrix3( void );
		Matrix3( const Matrix3<T>& other );
		Matrix3( T m00, T m01, T m02, 
				 T m10, T m11, T m12,
				 T m20, T m21, T m22 );
		Matrix3( const T* m );

		// Operators ///

		Matrix3<T> transposed() const;
		Matrix3<T>& transpose();

		// Methods ///
		T determinant() const;

	public:
		T m[3][3];
	};

	#include "matrix3.inl"

	typedef Matrix3<float> Matrix3f;
	typedef Matrix3<double> Matrix3d;
}
}