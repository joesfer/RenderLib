#pragma once

#include <math/algebra/vector/vector3.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>

namespace RenderLib {
namespace Math {

	template< typename T >
	class Point3 {
	public:
		Point3<T>() : x(0), y(0), z(0) {}
		Point3<T>( T X, T Y, T Z ) : x(X), y(Y), z(Z) {}
		Point3<T>( const Point3<T>& other ) : x( other.x ), y( other.y ), z( other.z ) {}
		Point3<T>( const Vector3<T>& other ) : x( other.x ), y( other.y ), z( other.z ) {}

		inline Point3<T>  operator + (const Point3<T>& p) const;
		inline Point3<T>  operator + (const Vector3<T>& v) const;
		inline Vector3<T> operator - (const Point3<T>& p) const;
		inline bool operator == (const Point3<T>& p) const;
		inline bool operator != (const Point3<T>& p) const;
		inline Point3<T> operator * (T f) const;
		inline Point3<T> operator / (T f) const;
		inline void operator -= (const Vector3<T>& v);
		inline void operator -= (const Point3<T>& p);
		inline void operator += (const Vector3<T>& v);
		inline void operator += (const Point3<T>& p);
		inline void operator *= (const T a);
		inline void operator /= (const T a);

		inline Point3<T> operator -() const;

		T operator[](int i) const;
		inline T &operator[] (const int i);

		inline static T distance(const Point3<T>& p1, const Point3<T>& p2);
		inline static T distanceSquared(const Point3<T>& p1, const Point3<T>& p2);

		inline T distanceTo( const Point3<T>& p2 ) const;
		inline T distanceToOrigin() const;
		inline T distanceSquaredTo( const Point3<T>& p2 ) const;
		inline T distanceSquaredToOrigin() const;

		inline bool equals( const Point3<T>& p, T epsilon ) const;

		inline static Point3<T> clamp(const Point3<T>& v, const Point3<T>& min, const Point3<T>& max );
		inline static Point3<T> clamp(const Point3<T>& v, T min, T max);
		inline static Point3<T> clamp(const Point3<T>& v, T max);

		inline Vector3<T> fromOrigin() const { return Vector3<T>(x,y,z); } // point-vector conversion
	
	public:		
		T x, y, z;
	};

	typedef Point3<float>  Point3f;
	typedef Point3<double> Point3d;

	#include "point3.inl"
}
}