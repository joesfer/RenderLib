
template< typename T >
inline Vector2<T> Vector2<T>::operator + (const Vector2<T>& v) const {
	return Vector2<T>(x + v.x, y + v.y );
}

template< typename T >
inline Vector2<T> Vector2<T>::operator + (const Point2<T>& p) const {
	return Vector2<T>(x + p.x, y + p.y );
}

template< typename T >
inline Vector2<T> Vector2<T>::operator - (const Vector2<T>& v) const {
	return Vector2<T>(x - v.x, y - v.y );
}

template< typename T >
inline Vector2<T> Vector2<T>::operator - (const Point2<T>& p) const {
	return Vector2<T>(x - p.x, y - p.y );
}

template< typename T >
inline Vector2<T> Vector2<T>::operator * (const T f) const {
	return Vector2<T>(x*f, y*f );
}

template< typename T >
inline Vector2<T> Vector2<T>::operator / (const T f) const {
	return Vector2<T>(x/f, y/f );
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator +=( const Vector2< T > &v ) {
	x += v.x;
	y += v.y;

	return *this;
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator -=( const Vector2< T > &v ) {
	x -= v.x;
	y -= v.y;

	return *this;
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator +=( const Point2< T > &p ) {
	x += p.x;
	y += p.y;

	return *this;
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator -=( const Point2< T > &p ) {
	x -= p.x;
	y -= p.y;

	return *this;
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator += (const T a) {
	x += a;
	y += a;

	return *this;
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator -= (const T a) {
	x -= a;
	y -= a;

	return *this;
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator *= (const T a) {
	x *= a;
	y *= a;

	return *this;
}

template< typename T >
inline Vector2< T >& Vector2< T >::operator /= (const T a) {
	x /= a;
	y /= a;

	return *this;
}

template< typename T >
inline Vector2<T> Vector2<T>::operator -() const {
	return Vector2<T>(-x, -y);
}

template< typename T >
inline T Vector2<T>::operator[](int i) const { 
	assert(i >= 0 && i <= 1);
	return (&x)[i]; 
}

template< typename T >
inline T& Vector2<T>::operator[] (int i) {
	assert(i >= 0 && i <= 1);
	return (&x)[i];
}

template< typename T >
inline bool Vector2<T>::operator == (const Vector2<T>& v) const {
	return x == v.x && y == v.y;
}

template< typename T >
inline bool Vector2<T>::operator != (const Vector2<T>& v) const {
	return x != v.x || y != v.y;
}

template< typename T >
inline static T Vector2<T>::dot(const Vector2<T>& v1, const Vector2<T>& v2) {
	return v1.x * v2.x + v1.y * v2.y;
}

template< typename T >
inline static Vector2<T> Vector2<T>::normalize(const Vector2<T>& v)  {
	T length = sqrt(v.x * v.x + v.y * v.y);
	assert(length > 0);
	return Vector2<T>(v.x / length, v.y / length);
}

template< typename T >
inline T Vector2<T>::normalize( void ) { 
	T length = sqrt(x * x + y * y);
	assert( length > 0 );
	if ( length > 0 ) {
		x /= length;
		y /= length;
	}
	return length;
}

template< typename T >
inline T Vector2<T>::length() const { 
	return sqrt(x * x + y * y); 
}	

template< typename T >
inline T Vector2<T>::lengthSquared() const { 
	return x * x + y * y; 
}	

template< typename T >
inline Vector2<T> Vector2<T>::clamp(const Vector2<T>& v, T m, T M) {
	return Vector2<T>( std::min( M, std::max( m, v.x ) ), std::min( M, std::max( m, v.y ) ) );
}

template< typename T >
inline Vector2<T> Vector2<T>::clamp(const Vector2<T>& v, const Vector2<T>& m, const Vector2<T>& M ) {
	return Vector2<T>( std::min( M.x, std::max( m.x, v.x ) ), std::min( M.y, std::max( m.y, v.y ) ) );
}

template< typename T >
inline bool Vector2< T >::isZero( const T epsilon ) const {
	return	abs( x ) <= epsilon && abs( y ) <= epsilon;
}

template< typename T >
inline bool Vector2<T>::equals( const Vector2<T>& p, const T epsilon ) const {
	return	abs( x - p.x ) <= epsilon &&
			abs( y - p.y ) <= epsilon;
}

// specializations for Vector2d ////////////////////////////////////////////////////

// according to the IEEE standards, a NaN number always returns false when compared against itself
#define _isnanf( f ) ( f != f )

template<>
inline bool Vector2<double>::isValid() const {
	return !_isnan( x ) && !_isnan( y )/* && _finite( x ) && _finite( y ) */;
}

template<>
inline Vector2<double> Vector2<double>::clamp(const Vector2<double>& v, double M) {
	return Vector2<double>::clamp( v, -DBL_MAX, M );
}

// specializations for Vector2f ////////////////////////////////////////////////////

template<>
inline bool Vector2<float>::isValid() const {
	return !_isnanf( x ) && !_isnanf( y )/* && _finitef( x ) && _finitef( y )*/;
}

template<>
inline Vector2<float> Vector2<float>::clamp(const Vector2<float>& v, float M) {
	return Vector2<float>::clamp( v, -FLT_MAX, M );
}

template<> 
inline float Vector2< float >::length() const { 
	return sqrtf(x * x + y * y); 
}	

template<>
inline Vector2<float> Vector2<float>::normalize( const Vector2< float >& v ) {
	float length = sqrtf(v.x * v.x + v.y * v.y);
	assert(length > 0);
	return Vector2< float >(v.x / length, v.y / length);
}

template<>
inline float Vector2<float>::normalize() {
	const float length = sqrtf(x * x + y * y);
	if ( length >= 0.00001f ) { 
		x /= length;
		y /= length;
	}
	return length;
}

template<>
inline bool Vector2<float>::isZero( const float epsilon ) const {
	return	fabsf( x ) <= epsilon && fabsf( y ) <= epsilon;
}

template<>
inline bool Vector2<float>::equals( const Vector2<float>& p, const float epsilon ) const {
	return	fabsf( x - p.x ) <= epsilon &&
			fabsf( y - p.y ) <= epsilon;
}