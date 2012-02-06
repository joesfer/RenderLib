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
