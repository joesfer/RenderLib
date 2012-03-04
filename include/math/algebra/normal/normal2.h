#pragma once

#include <math/algebra/vector/vector2.h>
#include <math.h>

/////////////////////////////////////////////////////////////////
// A normal is like a Vector, but stays unitary all the time
/////////////////////////////////////////////////////////////////

namespace RenderLib {
namespace Math {
	
	template<typename T>
	class Normal2 {
	public:
		inline Normal2<T>() : x(0), y(0) {};
		inline Normal2<T>( const T X, const T Y);
		inline Normal2<T>( const Vector2<T>& v);

		inline Normal2<T> operator + (const Normal2<T>& n) const;
		inline Normal2<T> operator - (const Normal2<T>& n) const;
		
		inline Vector2<T> operator *(const T f) const;
		inline T operator * (const Vector2<T>& v) const;
		inline T operator * (const Normal2<T>& n) const;

		inline bool operator == (const Normal2<T>& n) const;
		inline bool operator != (const Normal2<T>& n) const;
		
		inline void operator += (const Normal2<T>& n);
		inline void operator -= (const Normal2<T>& n);
		inline void operator *= (const T f);

		inline Normal2<T> operator -() const;

		inline T normalize();
		inline static Normal2<T> normalize(const Normal2<T>& n);

		inline bool equals( const Normal2<T>& p, T epsilon ) const;

	public:
		T x,y;
	};

	typedef Normal2<float>  Normal2f;
	typedef Normal2<double> Normal2d;

	#include "normal2.inl"
}
}