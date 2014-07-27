
template< typename T >
inline Normal3<T>::Normal3( const T X, const T Y, const T Z) {
	// build Normal3<T> making vector <x,y,z> unitary

	T length = sqrt(X*X + Y*Y + Z*Z);		
	if ( length > 0 ) {
		x = X / length; 
		y = Y / length; 
		z = Z / length;
	}
}

template< typename T >
inline Normal3<T>::Normal3( const Vector3<T>& v) {
	// build Normal3<T> making vector <x,y,z> unitary

	T length = v.length();

	if ( length > 0 ) {
		x = v.x / length;
		y = v.y / length;
		z = v.z / length;
	}
}

template< typename T >
inline Normal3<T> Normal3<T>::operator + (const Normal3<T>& n) const {
	return Normal3<T>(x + n.x, y + n.y, z + n.z);
}

template< typename T >
inline Normal3<T> Normal3<T>::operator - (const Normal3<T>& n) const {
	return Normal3<T>(x - n.x, y - n.y, z - n.z);
}

template< typename T >
inline Vector3<T> Normal3<T>::operator *(const T f) const {
	return Vector3<T>(x*f, y*f, z*f);
}

template< typename T >
inline bool Normal3<T>::operator == (const Normal3<T>& n) const {
	return x == n.x && y == n.y && z == n.z;
}

template< typename T >
inline bool Normal3<T>::operator != (const Normal3<T>& n) const {
	return x != n.x || y != n.y || z != n.z;
}

template< typename T >
inline void Normal3<T>::operator += (const Normal3<T>& n) {
	x += n.x; 
	y += n.y; 
	z += n.z;
}

template< typename T >
inline void Normal3<T>::operator -= (const Normal3<T>& n) {
	x -= n.x; 
	y -= n.y; 
	z -= n.z;
}

template< typename T >
inline void Normal3<T>::operator *= (const T f) {
	x *= f; 
	y *= f; 
	z *= f;
}

template< typename T >
inline Normal3<T> Normal3<T>::operator -() const {
	return Normal3<T>(-x, -y, -z);
}

template< typename T >
inline Normal3<T>& Normal3<T>::normalize() {
	T length = sqrt( x * x + y * y + z * z);
	assert(length != 0);
	T invLength = 1.0 / length;
	x *= invLength; 
	y *= invLength;
	z *= invLength;
	return *this;
}

template< typename T >
inline Normal3<T> Normal3<T>::normalize(const Normal3<T>& n) {
	T length = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
	assert(length != 0);
	T invLength = static_cast<T>(1.0) / length;
	return Normal3<T>(n.x * invLength, n.y * invLength, n.z * invLength);
}

template< typename T >
inline T Normal3<T>::dot(const Normal3<T>& n, const Vector3<T>& v) {
	return n.x*v.x + n.y*v.y + n.z*v.z;
}

template< typename T >
inline T Normal3<T>::dot(const Normal3<T>& n1, const Normal3<T>& n2) {
	return n1.x*n2.x + n1.y*n2.y + n1.z*n2.z;
}

template< typename T >
inline bool Normal3<T>::equals( const Normal3<T>& p, T epsilon ) const {
	return	abs( x - p.x ) <= epsilon &&
		abs( y - p.y ) <= epsilon &&
		abs( z - p.z ) <= epsilon;
}

template< typename T >
inline Normal3<T> Normal3<T>::cross(const Normal3<T>& n1, const Normal3<T>& n2) {
	return Normal3<T>((n1.y * n2.z) - (n1.z * n2.y),
		(n1.z * n2.x) - (n1.x * n2.z),
		(n1.x * n2.y) - (n1.y * n2.x));
}

// specialization for Normalf //////////////////////////////////////////////////////////////////////////

template<>
inline bool Normal3<float>::equals( const Normal3<float>& p, float epsilon ) const {
	return	fabs( x - p.x ) <= epsilon &&
		fabs( y - p.y ) <= epsilon &&
		fabs( z - p.z ) <= epsilon;
}

template<>
inline Normal3<float>& Normal3<float>::normalize() {
	float length = sqrtf( x * x + y * y + z * z);
	assert(length != 0);
	float invLength = 1.0f / length;
	x *= invLength; 
	y *= invLength;
	z *= invLength;
	return *this;
}
