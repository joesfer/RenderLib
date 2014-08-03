
template< typename T >
inline Normal2<T>::Normal2( const T X, const T Y) {
	// build Normal2<T> making vector <x,y> unitary

	T length = sqrt(X*X + Y*Y);		
	assert(length > 0);

	x = X / length; 
	y = Y / length; 
}

template< typename T >
inline Normal2<T>::Normal2( const Vector2<T>& v) {
	// build Normal2<T> making vector <x,y> unitary

	T length = v.Length();

	assert(length > 0);
	x = v.x / length;
	y = v.y / length;
}

template< typename T >
inline Normal2<T> Normal2<T>::operator + (const Normal2<T>& n) const {
	return Normal2<T>(x + n.x, y + n.y);
}

template< typename T >
inline Normal2<T> Normal2<T>::operator - (const Normal2<T>& n) const {
	return Normal2<T>(x - n.x, y - n.y);
}

template< typename T >
inline Vector2<T> Normal2<T>::operator *(const T f) const {
	return Vector2<T>(x*f, y*f);
}

template< typename T >
inline T Normal2<T>::operator * (const Vector2<T>& v) const {
	return x*v.x + y*v.y;
}

template< typename T >
inline T Normal2<T>::operator * (const Normal2<T>& n) const {
	return x*n.x + y*n.y;
}

template< typename T >
inline bool Normal2<T>::operator == (const Normal2<T>& n) const {
	return x == n.x && y == n.y;
}

template< typename T >
inline bool Normal2<T>::operator != (const Normal2<T>& n) const {
	return x != n.x || y != n.y;
}

template< typename T >
inline void Normal2<T>::operator += (const Normal2<T>& n) {
	x += n.x; 
	y += n.y; 
}

template< typename T >
inline void Normal2<T>::operator -= (const Normal2<T>& n) {
	x -= n.x; 
	y -= n.y; 
}

template< typename T >
inline void Normal2<T>::operator *= (const T f) {
	x *= f; 
	y *= f; 
}

template< typename T >
inline Normal2<T> Normal2<T>::operator -() const {
	return Normal2<T>(-x, -y);
}

template< typename T >
inline T Normal2<T>::normalize() {
	T length = sqrt( x * x + y * y);
	assert(length != 0);
	T invLength = 1.0 / length;
	x *= invLength; 
	y *= invLength;
	return length;
}

template< typename T >
inline /*static*/ Normal2<T> Normal2<T>::normalize(const Normal2<T>& n) {
	T length = sqrt(n.x*n.x + n.y*n.y);
	assert(length != 0);
	T invLength = static_cast<T>(1.0) / length;
	return Normal2<T>(n.x * invLength, n.y * invLength);
}

template< typename T >
inline bool Normal2<T>::equals( const Normal2<T>& p, T epsilon ) const {
	return	fabs( x - p.x ) <= epsilon &&
		fabs( y - p.y ) <= epsilon;
}

// specialization for Normalf //////////////////////////////////////////////////////////////////////////

template<>
inline bool Normal2<float>::equals( const Normal2<float>& p, float epsilon ) const {
	return	fabs( x - p.x ) <= epsilon &&
		fabs( y - p.y ) <= epsilon;
}

template<>
inline float Normal2<float>::normalize() {
	float length = sqrtf( x * x + y * y );
	assert(length != 0);
	float invLength = 1.0f / length;
	x *= invLength; 
	y *= invLength;
	return length;
}
