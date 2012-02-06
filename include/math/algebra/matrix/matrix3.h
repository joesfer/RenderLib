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
#include <memory.h>

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
