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
