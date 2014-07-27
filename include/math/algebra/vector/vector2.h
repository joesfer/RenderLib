#pragma once

#include <math.h>
#include <assert.h>
#include <float.h>
#include <stdlib.h>
#include <algorithm>

namespace RenderLib {
namespace Math {

	template<typename T>
	class Normal2;

	template<typename T>
	class Point2;

	template<typename T>
	class Vector2 {
	public:

		inline Vector2<T>() : x(0), y(0) {}
		inline Vector2<T>(const T X, const T Y ) : x(X), y(Y) {}
		inline Vector2<T>( const Normal2<T>& n ) : x(n.x), y(n.y) {}

		inline Vector2<T> operator + (const Vector2<T>& v) const;
		inline Vector2<T> operator + (const Point2<T>& p) const;
		inline Vector2<T> operator - (const Vector2<T>& v) const;
		inline Vector2<T> operator - (const Point2<T>& p) const;
		inline Vector2<T> operator * (const T f) const;
		inline Vector2<T> operator / (const T f) const;

		inline Vector2<T>& operator += (const Vector2<T>& v);
		inline Vector2<T>& operator += (const Point2<T>& p);
		inline Vector2<T>& operator -= (const Vector2<T>& v);
		inline Vector2<T>& operator -= (const Point2<T>& p);

		inline Vector2<T>& operator += (const T a);
		inline Vector2<T>& operator -= (const T a);
		inline Vector2<T>& operator *= (const T a);
		inline Vector2<T>& operator /= (const T a);

		inline Vector2<T> operator -() const;

		inline T operator[](int i) const;
		inline T &operator[] (int i);

		inline bool operator == (const Vector2<T>& v) const;
		inline bool operator != (const Vector2<T>& v) const;

		inline T normalize( void );
		inline T length() const;
		inline T lengthSquared() const;

		inline bool isValid() const;

		inline bool isZero( const T epsilon = FLT_MIN ) const;

		inline bool equals( const Vector2<T>& p, const T epsilon ) const;

		inline static T dot(const Vector2<T>& v1, const Vector2<T>& v2);
		inline static Vector2<T> normalize(const Vector2<T>& v);
		inline static Vector2<T> clamp(const Vector2<T>& v, const Vector2<T>& min, const Vector2<T>& max);
		inline static Vector2<T> clamp(const Vector2<T>& v, T min, T max);
		inline static Vector2<T> clamp(const Vector2<T>& v, T max);

	public:

		T x,y;

	};

	typedef Vector2<float> Vector2f;
	typedef Vector2<double> Vector2d;

	#include "vector2.inl"
}
}
