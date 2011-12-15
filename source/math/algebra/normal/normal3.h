#pragma once

#include <math/algebra/vector/vector3.h>
#include <math.h>

/////////////////////////////////////////////////////////////////
// A normal is like a Vector3, but stays unitary all the time
/////////////////////////////////////////////////////////////////

namespace RenderLib {
namespace Math {

	template<typename T>
	class Normal3 {
	public:
		inline Normal3<T>() : x(0), y(0), z(0) {};
		inline Normal3<T>( const T X, const T Y, const T Z);
		inline Normal3<T>( const Vector3<T>& v);

		inline Normal3<T> operator + (const Normal3<T>& n) const;
		inline Normal3<T> operator - (const Normal3<T>& n) const;
		inline Vector3<T> operator *(const T f) const;
		
		inline bool operator == (const Normal3<T>& n) const;
		inline bool operator != (const Normal3<T>& n) const;
		inline void operator += (const Normal3<T>& n);
		inline void operator -= (const Normal3<T>& n);
		inline void operator *= (const T f);
		
		inline Normal3<T> operator -() const;

		inline Normal3<T>& normalize();
		inline static Normal3<T> normalize(const Normal3<T>& n);

		inline static T dot(const Normal3<T>& n1, const Normal3<T>& n2);
		inline static T dot(const Normal3<T>& n, const Vector3<T>& v);
		inline static Normal3<T> cross(const Normal3<T>& n1, const Normal3<T>& n2);

		inline bool equals( const Normal3<T>& p, T epsilon ) const;

	public:
		T x,y,z;
	};

	typedef Normal3<float>  Normal3f;
	typedef Normal3<double> Normal3d;

	#include "Normal3.inl"
}
}