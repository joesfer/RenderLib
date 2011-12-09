#pragma once
#include <math.h>
#include <assert.h>
#include <float.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>

namespace RenderLib {
namespace Math {

	template<class T>
	class Normal3;

	template<class T>
	class Point3;

	template<class T>
	class Vector3 {
	public:
		inline Vector3<T>() : x(0), y(0), z(0) {}
		inline Vector3<T>(const T X, const T Y, const T Z) : x(X), y(Y), z(Z) {}
		inline Vector3<T>( const Normal3<T>& n ) : x(n.x), y(n.y), z(n.z) {}

		inline Vector3<T> operator + (const Vector3<T>& v) const;
		inline Vector3<T> operator + (const Point3<T>& p) const;
		inline Vector3<T> operator - (const Vector3<T>& v) const;
		inline Vector3<T> operator - (const Point3<T>& p) const;
		inline Vector3<T> operator * (const T f) const;
		inline Vector3<T> operator / (const T f) const;

		inline Vector3<T>& operator += (const Vector3<T>& v);
		inline Vector3<T>& operator += (const Point3<T>& p);
		inline Vector3<T>& operator -= (const Vector3<T>& v);
		inline Vector3<T>& operator -= (const Point3<T>& p);

		inline Vector3<T>& operator += (const T a);
		inline Vector3<T>& operator -= (const T a);
		inline Vector3<T>& operator *= (const T a);
		inline Vector3<T>& operator /= (const T a);

		inline Vector3<T> operator -() const;

		inline T operator[](int i) const;
		inline T &operator[] (int i);

		inline bool operator == (const Vector3<T>& v) const;
		inline bool operator != (const Vector3<T>& v) const;

		inline T normalize( void );
		inline Vector3<T>  cross( const Vector3<T>& v ) const;
		inline T length() const;
		inline T lengthSquared() const;

		inline bool isValid() const;

		inline bool isZero( const T epsilon = FLT_MIN ) const;

		inline bool equals( const Vector3<T>& p, const T epsilon ) const;

		inline static Vector3<T> normalize(const Vector3<T>& v);
		inline static T dot(const Vector3<T>& v1, const Vector3<T>& v2);
		inline static Vector3<T> cross(const Vector3<T>& v1, const Vector3<T>& v2);
		inline static Vector3<T> clamp(const Vector3<T>& v, const Vector3<T>& min, const Vector3<T>& max);
		inline static Vector3<T> clamp(const Vector3<T>& v, T min, T max);
		inline static Vector3<T> clamp(const Vector3<T>& v, T max);

	public:

		T x,y,z;

	};

	typedef Vector3<float> Vector3f;
	typedef Vector3<double> Vector3d;

	#include "vector3.inl"
}
}