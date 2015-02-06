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
#include <math.h>
#include <assert.h>
#include <math/constants.h>
#include <math/algebra/point/point2.h>
#include <math/algebra/vector/vector3.h>
#include <math/algebra/point/point3.h>
#include <math/algebra/matrix/matrix3.h>
#include <math/algebra/matrix/matrix4.h>
#include <math.h>

namespace RenderLib {
namespace Geometry {

	#define DEG2RAD(x) (x * RenderLib::Math::PI / 180)

	// ================================================================================================
	// Barycentric texture coordinate functions
	// Get the *SIGNED* area of a triangle required for barycentric
	// ================================================================================================
	template< typename T >
	T BarycentricTriangleArea( const RenderLib::Math::Normal3<T> &normal, const RenderLib::Math::Point3<T> &a, const RenderLib::Math::Point3<T> &b, const RenderLib::Math::Point3<T> &c ) {
                using namespace RenderLib::Math;
		Vector3<T>	v1, v2;
		Vector3<T>	cross;
		T	area;

		v1 = b - a;
		v2 = c - a;
		cross = Vector3<T>::cross(v1, v2);
		area = 0.5f * Vector3<T>::dot(cross, normal);

		return( area );
	}

	// Triangle Area using Heron's formula
	template< typename T >
	T triangleArea( const RenderLib::Math::Point3<T> &a, const RenderLib::Math::Point3<T> &b, const RenderLib::Math::Point3<T> &c ) {
		const T sa = b.distanceTo( c );
		const T sb = a.distanceTo( c );
		const T sc = a.distanceTo( b );
		return sqrt( ( sa + sb - sc ) * ( sa - sb + sc ) * ( -sa + sb + sc ) * ( sa + sb + sc ) ) / 4;
	}

	// Obtain sphere that passes through 4 non coplanar points
	// References: http://mathworld.wolfram.com/Sphere.html  http://local.wasp.uwa.edu.au/~pbourke/geometry/spherefrom4/
	template< typename T >
	bool SphereFrom4Points( const RenderLib::Math::Point3<T>& p0, const RenderLib::Math::Point3<T>& p1, const RenderLib::Math::Point3<T>& p2, const RenderLib::Math::Point3<T>& p3, 
							RenderLib::Math::Point3<T>& center, T& radius ) {
		using namespace RenderLib::Math;
		/*
			| ( x^2 +  y^2 +  z^2)  x   y   z   1  |
			|                                      |
			| (x1^2 + y1^2 + z1^2)  x1  y1  z1  1  |
			|                                      |
			| (x2^2 + y2^2 + z2^2)  x2  y2  z2  1  | = 0
			|                                      |
			| (x3^2 + y3^2 + z3^2)  x3  y3  z3  1  |
			|                                      |
			| (x4^2 + y4^2 + z4^2)  x4  y4  z4  1  |
		*/
		
		// Minors
		Matrix4<T> _minor(	p0.x, p0.y, p0.z, static_cast<T>(1),
							p1.x, p1.y, p1.z, static_cast<T>(1),
							p2.x, p2.y, p2.z, static_cast<T>(1),
							p3.x, p3.y, p3.z, static_cast<T>(1) );
		T m11 = _minor.determinant();

		if ( abs( m11 ) < 1e-5 ) { 
			return false;
		}

		_minor.m[0][0] = p0.x * p0.x + p0.y * p0.y + p0.z * p0.z; _minor.m[0][1] = p0.y; _minor.m[0][2] = p0.z; _minor.m[0][3] = 1;
		_minor.m[1][0] = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z; _minor.m[1][1] = p1.y; _minor.m[1][2] = p1.z; _minor.m[1][3] = 1;
		_minor.m[2][0] = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z; _minor.m[2][1] = p2.y; _minor.m[2][2] = p2.z; _minor.m[2][3] = 1;
		_minor.m[3][0] = p3.x * p3.x + p3.y * p3.y + p3.z * p3.z; _minor.m[3][1] = p3.y; _minor.m[3][2] = p3.z; _minor.m[3][3] = 1;
		T m12 = _minor.determinant();

		_minor.m[0][0] = p0.x * p0.x + p0.y * p0.y + p0.z * p0.z; _minor.m[0][1] = p0.x; _minor.m[0][2] = p0.z; _minor.m[0][3] = 1;
		_minor.m[1][0] = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z; _minor.m[1][1] = p1.x; _minor.m[1][2] = p1.z; _minor.m[1][3] = 1;
		_minor.m[2][0] = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z; _minor.m[2][1] = p2.x; _minor.m[2][2] = p2.z; _minor.m[2][3] = 1;
		_minor.m[3][0] = p3.x * p3.x + p3.y * p3.y + p3.z * p3.z; _minor.m[3][1] = p3.x; _minor.m[3][2] = p3.z; _minor.m[3][3] = 1;
		T m13 = _minor.determinant();

		_minor.m[0][0] = p0.x * p0.x + p0.y * p0.y + p0.z * p0.z; _minor.m[0][1] = p0.x; _minor.m[0][2] = p0.y; _minor.m[0][3] = 1;
		_minor.m[1][0] = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z; _minor.m[1][1] = p1.x; _minor.m[1][2] = p1.y; _minor.m[1][3] = 1;
		_minor.m[2][0] = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z; _minor.m[2][1] = p2.x; _minor.m[2][2] = p2.y; _minor.m[2][3] = 1;
		_minor.m[3][0] = p3.x * p3.x + p3.y * p3.y + p3.z * p3.z; _minor.m[3][1] = p3.x; _minor.m[3][2] = p3.y; _minor.m[3][3] = 1;
		T m14 = _minor.determinant();

		_minor.m[0][0] = p0.x * p0.x + p0.y * p0.y + p0.z * p0.z; _minor.m[0][1] = p0.x; _minor.m[0][2] = p0.y; _minor.m[0][3] = p0.z;
		_minor.m[1][0] = p1.x * p1.x + p1.y * p1.y + p1.z * p1.z; _minor.m[1][1] = p1.x; _minor.m[1][2] = p1.y; _minor.m[1][3] = p1.z;
		_minor.m[2][0] = p2.x * p2.x + p2.y * p2.y + p2.z * p2.z; _minor.m[2][1] = p2.x; _minor.m[2][2] = p2.y; _minor.m[2][3] = p2.z;
		_minor.m[3][0] = p3.x * p3.x + p3.y * p3.y + p3.z * p3.z; _minor.m[3][1] = p3.x; _minor.m[3][2] = p3.y; _minor.m[3][3] = p3.z;
		T m15 = _minor.determinant();

		
		center.x =  static_cast<T>(0.5) * m12 / m11;
		center.y = -static_cast<T>(0.5) * m13 / m11;
		center.z =  static_cast<T>(0.5) * m14 / m11;
		
		radius = sqrt( center.x * center.x + center.y * center.y + center.z * center.z - m15/m11 );

		assert( abs( center.distanceTo( p0 ) - radius ) < static_cast<T>(0.1) );
		assert( abs( center.distanceTo( p1 ) - radius ) < static_cast<T>(0.1) );
		assert( abs( center.distanceTo( p2 ) - radius ) < static_cast<T>(0.1) );
		assert( abs( center.distanceTo( p3 ) - radius ) < static_cast<T>(0.1) );

		return true;
	}


	// Calculate the circle that passes through 3 coplanar points
	// http://mathworld.wolfram.com/Circle.html
	template< typename T >
	bool circleFrom3Points( const RenderLib::Math::Point2<T>& p0, const RenderLib::Math::Point2<T>& p1, const RenderLib::Math::Point2<T>& p2,
							RenderLib::Math::Point2<T>& center, T& radius ) {
		using namespace RenderLib::Math;

		const float a = Matrix3< T >( p0.x, p0.y, 1, 
                                              p1.x, p1.y, 1,
                                              p2.x, p2.y, 1 ).determinant();
		if ( abs( a ) < 1e-5 ) {
			return false;
		}
		const float d = -Matrix3< T >( p0.x * p0.x + p0.y * p0.y, p0.y, 1, 
                                               p1.x * p1.x + p1.y * p1.y, p1.y, 1,
                                               p2.x * p2.x + p2.y * p2.y, p2.y, 1 ).determinant();

		const float e = Matrix3< T >( p0.x * p0.x + p0.y * p0.y, p0.x, 1, 
                                              p1.x * p1.x + p1.y * p1.y, p1.x, 1,
                                              p2.x * p2.x + p2.y * p2.y, p2.x, 1 ).determinant();

		const float f = -Matrix3< T >( p0.x * p0.x + p0.y * p0.y, p0.x, p0.y, 
                                               p1.x * p1.x + p1.y * p1.y, p1.x, p1.y,
                                               p2.x * p2.x + p2.y * p2.y, p2.x, p2.y ).determinant();

		center.x = - d / ( 2 * a );
		center.y = - e / ( 2 * a );
		radius = sqrt( ( d * d + e * e ) / ( 4 * a * a ) - f / a );

		return true;
	}

	// Calculate the sphere which equator passes through 3 coplanar points
	template< typename T >
	bool sphereFrom3Points( const RenderLib::Math::Point3<T>& a, const RenderLib::Math::Point3<T>& b, const RenderLib::Math::Point3<T>& c,
							RenderLib::Math::Point3<T>& center, T& radius ) {
		using namespace RenderLib::Math;

		// project points to a plane
		const Vector3<T> ab = Vector3<T>::normalize(b-a);
		const Vector3<T> ac = Vector3<T>::normalize(c-a);
		const Vector3<T> planeNormal = Vector3<T>::cross(ab,ac);
		const Vector3<T>& planeOX = ab;
		const Vector3<T> planeOY = Vector3<T>::normalize(Vector3<T>::cross(planeOX, planeNormal)); // renormalize to correct imprecissions

		Point2<T> projA(0,0);
		Point2<T> projB( Vector3<T>::dot(planeOX, b - a), Vector3<T>::dot(planeOY, b - a));
		Point2<T> projC( Vector3<T>::dot(planeOX, c - a), Vector3<T>::dot(planeOY, c - a));

		// find circle passing through projected points
		Point2<T> projCenter; float projRadius;
		if (!circleFrom3Points( projA, projB, projC, projCenter, projRadius )) return false;
#ifdef DEBUG
		assert( fabsf((projCenter - projA ).length() - projRadius) < 1e-3f );
#endif
		// define sphere from circle
		center = a + planeOX * projCenter.x + planeOY * projCenter.y;
		radius = projRadius;

#ifdef DEBUG
		// debug sanity check //
		float fa = fabsf((center - a ).length() - radius );
		float fb = fabsf((center - b ).length() - radius );
		float fc = fabsf((center - c ).length() - radius );
		assert( fabsf((center - a ).length() - radius) < 1e-2f );
		assert( fabsf((center - b ).length() - radius) < 1e-2f );
		assert( fabsf((center - c ).length() - radius) < 1e-2f );
#endif
		return true;
	}
	
	template<typename T>
	float Orient( const RenderLib::Math::Vector2f& a, const RenderLib::Math::Vector2f& b, const RenderLib::Math::Vector2f& c )
	{
		using namespace RenderLib::Math;
		return Matrix3<T>( a.x, a.y, 1, 
						   b.x, b.y, 1,
						   c.x, c.y, 1 ).determinant();
	}

	template<typename T>
	float inCircle( const RenderLib::Math::Vector2<T>& a, const RenderLib::Math::Vector2<T>& b, const RenderLib::Math::Vector2<T>& c, const RenderLib::Math::Vector2<T>& p )
	{
		using namespace RenderLib::Math;
		const float det = Matrix4<T>( a.x, a.y, a.x * a.x + a.y * a.y, 1, 
                                              b.x, b.y, b.x * b.x + b.y * b.y, 1,
                                              c.x, c.y, c.x * c.x + c.y * c.y, 1,
                                              p.x, p.y, p.x * p.x + p.y * p.y, 1 ).determinant();
		if ( Orient<T>( a, b, c ) > 0 ) {
			return det;
		} else {
			return -det;
		}
	}

	inline float scalarTriple( const RenderLib::Math::Point3f& u, const RenderLib::Math::Point3f& v, const RenderLib::Math::Point3f& w ) {
		return RenderLib::Math::Vector3f::dot(RenderLib::Math::Vector3f::cross(u.fromOrigin(),v.fromOrigin()), w.fromOrigin());
	}
}
}
