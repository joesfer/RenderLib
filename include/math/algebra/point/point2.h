#pragma once

#include <math/algebra/vector/vector2.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>

namespace RenderLib {
namespace Math {

	template< typename T >
	class Point2 {
	public:
		Point2<T>() : x(0), y(0) {}
		Point2<T>( T X, T Y ) : x(X), y(Y) {}
		Point2<T>( const Point2<T>& other ) : x( other.x ), y( other.y ) {}
		Point2<T>( const Vector2<T>& other ) : x( other.x ), y( other.y ) {}

		inline Point2<T>  operator + (const Point2<T>& p) const;
		inline Point2<T>  operator + (const Vector2<T>& v) const;
		inline Vector2<T> operator - (const Point2<T>& p) const;
		inline bool operator == (const Point2<T>& p) const;
		inline bool operator != (const Point2<T>& p) const;
		inline Point2<T> operator * (T f) const;
		inline Point2<T> operator / (T f) const;
		inline void operator -= (const Vector2<T>& v);
		inline void operator -= (const Point2<T>& p);
		inline void operator += (const Vector2<T>& v);
		inline void operator += (const Point2<T>& p);
		inline void operator *= (const T a);
		inline void operator /= (const T a);

		inline Point2<T> operator -() const;

		T operator[](int i) const;
		inline T &operator[] (const int i);

		inline static T distance(const Point2<T>& p1, const Point2<T>& p2);
		inline static T distanceSquared(const Point2<T>& p1, const Point2<T>& p2);

		inline T distanceTo( const Point2<T>& p2 ) const;
		inline T distanceToOrigin() const;
		inline T distanceSquaredTo( const Point2<T>& p2 ) const;
		inline T distanceSquaredToOrigin() const;

		inline bool equals( const Point2<T>& p, T epsilon ) const;

		inline static Point2<T> clamp(const Point2<T>& v, const Point2<T>& min, const Point2<T>& max );
		inline static Point2<T> clamp(const Point2<T>& v, T min, T max);
		inline static Point2<T> clamp(const Point2<T>& v, T max);

	public:		
		T x, y;
	};

	typedef Point2<float>  Point2f;
	typedef Point2<double> Point2d;

#include "point2.inl"

}
}