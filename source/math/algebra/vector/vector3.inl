
template< typename T >
inline Vector3<T> Vector3<T>::operator + (const Vector3<T>& v) const {
	return Vector3<T>(x + v.x, y + v.y, z + v.z);
}

template< typename T >
inline Vector3<T> Vector3<T>::operator + (const Point3<T>& p) const {
	return Vector3<T>(x + p.x, y + p.y, z + p.z);
}

template< typename T >
inline Vector3<T> Vector3<T>::operator - (const Vector3<T>& v) const {
	return Vector3<T>(x - v.x, y - v.y, z - v.z);
}

template< typename T >
inline Vector3<T> Vector3<T>::operator - (const Point3<T>& p) const {
	return Vector3<T>(x - p.x, y - p.y, z - p.z);
}

template< typename T >
inline Vector3<T> Vector3<T>::operator * (const T f) const {
	return Vector3<T>(x*f, y*f, z*f);
}

template< typename T >
inline Vector3<T> Vector3<T>::operator / (const T f) const {
	return Vector3<T>(x/f, y/f, z/f);
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator +=( const Vector3< T > &v ) {
	x += v.x;
	y += v.y;
	z += v.z;

	return *this;
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator -=( const Vector3< T > &v ) {
	x -= v.x;
	y -= v.y;
	z -= v.z;

	return *this;
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator +=( const Point3< T > &p ) {
	x += p.x;
	y += p.y;
	z += p.z;

	return *this;
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator -=( const Point3< T > &p ) {
	x -= p.x;
	y -= p.y;
	z -= p.z;

	return *this;
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator += (const T a) {
	x += a;
	y += a;
	z += a;

	return *this;
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator -= (const T a) {
	x -= a;
	y -= a;
	z -= a;

	return *this;
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator *= (const T a) {
	x *= a;
	y *= a;
	z *= a;

	return *this;
}

template< typename T >
inline Vector3< T >& Vector3< T >::operator /= (const T a) {
	x /= a;
	y /= a;
	z /= a;

	return *this;
}

template< typename T >
inline Vector3<T> Vector3<T>::operator -() const {
	return Vector3<T>(-x, -y, -z);
}

template< typename T >
inline T Vector3<T>::operator[](int i) const { 
	assert(i >= 0 && i <= 2);
	return (&x)[i]; 
}

template< typename T >
inline T& Vector3<T>::operator[] (int i) {
	assert(i >= 0 && i <= 2);
	return (&x)[i];
}

template< typename T >
inline bool Vector3<T>::operator == (const Vector3<T>& v) const {
	return x == v.x && y == v.y && z == v.z;
}

template< typename T >
inline bool Vector3<T>::operator != (const Vector3<T>& v) const {
	return x != v.x || y != v.y || z != v.z;
}

template< typename T >
/*static*/ inline  Vector3<T> Vector3<T>::normalize(const Vector3<T>& v)  {
	T length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	assert(length > 0);
	return Vector3<T>(v.x / length, v.y / length, v.z / length);
}

template< typename T >
inline T Vector3<T>::normalize( void ) { 
	T length = sqrt(x * x + y * y + z * z);
	assert( length > 0 );
	if ( length > 0 ) {
		x /= length;
		y /= length;
		z /= length;
	}
	return length;
}

template< typename T >
/*static*/ inline T Vector3<T>::dot(const Vector3<T>& v1, const Vector3<T>& v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; // dot product
}

template< typename T >
/*static*/ inline Vector3<T> Vector3<T>::cross(const Vector3<T>& v1, const Vector3<T>& v2) {
	return Vector3<T>((v1.y * v2.z) - (v1.z * v2.y),
		(v1.z * v2.x) - (v1.x * v2.z),
		(v1.x * v2.y) - (v1.y * v2.x));
}

template< typename T >
inline Vector3<T> Vector3<T>::cross( const Vector3<T>& v ) const {
	return Cross( *this, v );
}

template< typename T >
inline T Vector3<T>::length() const { 
	return sqrt(x * x + y * y + z * z); 
}	

template< typename T >
inline T Vector3<T>::lengthSquared() const { 
	return x * x + y * y + z * z; 
}	

template< typename T >
inline Vector3<T> Vector3<T>::clamp(const Vector3<T>& v, T m, T M) {
	return Vector3<T>( std::min( M, std::max( m, v.x ) ), std::min( M, std::max( m, v.y ) ), std::min( M, std::max( m, v.z ) ) );
}

template< typename T >
inline Vector3<T> Vector3<T>::clamp(const Vector3<T>& v, const Vector3<T>& m, const Vector3<T>& M ) {
	return Vector3<T>( std::min( M.x, std::max( m.x, v.x ) ), std::min( M.y, std::max( m.y, v.y ) ), std::min( M.z, std::max( m.z, v.z ) ) );
}

template< typename T >
inline bool Vector3< T >::isZero( const T epsilon ) const {
	return	abs( x ) <= epsilon && abs( y ) <= epsilon && abs( z ) <= epsilon;
}

template< typename T >
inline bool Vector3<T>::equals( const Vector3<T>& p, const T epsilon ) const {
	return	abs( x - p.x ) <= epsilon &&
			abs( y - p.y ) <= epsilon &&
			abs( z - p.z ) <= epsilon;
}

// specializations for vector3d ////////////////////////////////////////////////////

// according to the IEEE standards, a NaN number always returns false when compared against itself
#define _isnan( f ) ( f != f )

template<>
inline bool Vector3<double>::isValid() const {
	return !_isnan( x ) && !_isnan( y ) && !_isnan( z )/* && _finite( x ) && _finite( y ) && _finite( z )*/;
}

template<>
inline Vector3<double> Vector3<double>::clamp(const Vector3<double>& v, double M) {
	return Vector3<double>::clamp( v, -DBL_MAX, M );
}

// specializations for vector3f ////////////////////////////////////////////////////

template<>
inline bool Vector3<float>::isValid() const {
	return !_isnan( x ) && !_isnan( y ) && !_isnan( z )/* && _finitef( x ) && _finitef( y ) && _finitef( z )*/;
}

template<>
inline Vector3<float> Vector3<float>::clamp(const Vector3<float>& v, float M) {
	return Vector3<float>::clamp( v, -FLT_MAX, M );
}

template<> 
inline float Vector3< float >::length() const { 
	return sqrtf(x * x + y * y + z * z); 
}	

template<>
inline Vector3<float> Vector3<float>::normalize( const Vector3< float >& v ) {
	float length = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
	assert(length > 0);
	return Vector3< float >(v.x / length, v.y / length, v.z / length);
}

template<>
inline float Vector3<float>::normalize() {
	const float length = sqrtf(x * x + y * y + z * z);
	if ( length >= 0.00001f ) { 
		x /= length;
		y /= length;
		z /= length;
	}
	return length;
}

template<>
inline bool Vector3<float>::isZero( const float epsilon ) const {
	return	fabsf( x ) <= epsilon && fabsf( y ) <= epsilon && fabsf( z ) <= epsilon;
}

template<>
inline bool Vector3<float>::equals( const Vector3<float>& p, const float epsilon ) const {
	return	fabsf( x - p.x ) <= epsilon &&
			fabsf( y - p.y ) <= epsilon &&
			fabsf( z - p.z ) <= epsilon;
}