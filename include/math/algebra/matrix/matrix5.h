#pragma once

namespace RenderLib {
namespace Math {

	template< typename T >
	class Matrix5 {
	public:
		// Constructors ///
		Matrix5( void );
		Matrix5( const Matrix5<T>& other );
		Matrix5( T m00, T m01, T m02, T m03, T m04, 
				 T m10, T m11, T m12, T m13, T m14,
				 T m20, T m21, T m22, T m23, T m24,
				 T m30, T m31, T m32, T m33, T m34,
				 T m40, T m41, T m42, T m43, T m44 );
		Matrix5( const T* m );

		// Operators ///

		// TODO

		// Methods ///
		T determinant() const;

	public:
		T m[5][5];
	};

	#include "matrix5.inl"

	typedef Matrix5<float> Matrix5f;
	typedef Matrix5<double> Matrix5d;
}
}
