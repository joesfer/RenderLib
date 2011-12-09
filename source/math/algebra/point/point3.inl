
template< class T >
inline Point3<T>  Point3<T>::operator + (const Point3<T>& p) const { 
	return Point3<T>(x + p.x, y + p.y, z + p.z); 
}

template< class T >
inline Point3<T>  Point3<T>::operator + (const Vector3<T>& v) const { 
	return Point3<T>(x + v.x, y + v.y, z + v.z); 
}

template< class T >
inline Vector3<T> Point3<T>::operator - (const Point3<T>& p) const { 
	return Vector3<T>(x - p.x, y - p.y, z - p.z); // that's how we usually define a vector, from two points
}

template< class T >
inline bool Point3<T>::operator == (const Point3<T>& p) const { 
	return x == p.x && y == p.y && z == p.z; 
}

template< class T >
inline bool Point3<T>::operator != (const Point3<T>& p) const { 
	return x != p.x || y != p.y || z != p.z; 
}

template< class T >
inline Point3<T> Point3<T>::operator * (T f) const { 
	return Point3<T>( x*f, y*f, z*f ); 
}

template< class T >
inline Point3<T> Point3<T>::operator / (T f) const { 
	assert(f != 0); return (*this) * ( 1.0f / f ); 
}

template< class T >
inline void Point3<T>::operator -= (const Vector3<T>& v) { 
	x -= v.x; 
	y -= v.y; 
	z -= v.z; 
}

template< class T >
inline void Point3<T>::operator -= (const Point3<T>& p) { 
	x -= p.x; 
	y -= p.y; 
	z -= p.z; 
}

template< class T >
inline void Point3<T>::operator += (const Vector3<T>& v) { 
	x += v.x; 
	y += v.y; 
	z += v.z; 
}

template< class T >
inline void Point3<T>::operator += (const Point3<T>& p) { 
	x += p.x; 
	y += p.y; 
	z += p.z; 
}

template< class T >
inline void Point3<T>::operator *= (const T a)	{ 
	x *= a; 
	y *= a; 
	z *= a; 
}

template< class T >
inline void Point3<T>::operator /= (const T a) { 
	x /= a; 
	y /= a; 
	z /= a; 
}

template< class T >
inline Point3<T> Point3<T>::operator -() const {
	return Point3<T>( -x, -y, -z );
}

template< class T >
T Point3<T>::operator[](int i) const { 
	assert(i >= 0 && i <= 2);
	return (&x)[i]; 
}	

template< class T >
inline T& Point3<T>::operator[] (const int i) {
	assert(i >= 0 && i <= 2);
	return (&x)[i];
}

template< class T >
inline T Point3<T>::distance(const Point3<T>& p1, const Point3<T>& p2) { 
	return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z)); 
}

template< class T >
inline T Point3<T>::distanceSquared(const Point3<T>& p1, const Point3<T>& p2) { 
	return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z)); 
}

template< class T >
inline T Point3<T>::distanceTo( const Point3<T>& p2 ) const { 
	return Distance( *this, p2 ); 
}

template< class T >
inline T Point3<T>::distanceToOrigin() const { 
	return Distance( *this, Point3<T>() ); 
}

template< class T >
inline T Point3<T>::distanceSquaredTo( const Point3<T>& p2 ) const { 
	return DistanceSquared( *this, p2 ); }

template< class T >
inline T Point3<T>::distanceSquaredToOrigin() const { 
	return DistanceSquared( *this, Point3<T>() ); 
}

template< class T >
inline Point3<T> Point3<T>::clamp(const Point3<T>& v, T m, T M) {
	return Point3<T>( std::min( M, std::max( m, v.x ) ), std::min( M, std::max( m, v.y ) ), std::min( M, std::max( m, v.z ) ) );
}

template< class T >
inline Point3<T> Point3<T>::clamp(const Point3<T>& v, const Point3<T>& m, const Point3<T>& M ) {
	return Point3<T>( std::min( M.x, std::max( m.x, v.x ) ), std::min( M.y, std::max( m.y, v.y ) ), std::min( M.z, std::max( m.z, v.z ) ) );
}

template< class T >
inline bool Point3<T>::equals( const Point3<T>& p, T epsilon ) const {
	return	abs( x - p.x ) <= epsilon &&
			abs( y - p.y ) <= epsilon &&
			abs( z - p.z ) <= epsilon;
}

// specializations for Point3d ////////////////////////////////////////////////////

template<>
inline Point3<double> Point3<double>::clamp(const Point3<double>& v, double M) {
	return Point3<double>::clamp( v, -DBL_MAX, M );
}

// specializations for Point3f ////////////////////////////////////////////////////

template<> 
inline float Point3<float>::distance( const Point3<float>& p1, const Point3<float>& p2 ) { 
	return sqrtf((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z)); 
}	

template<>
inline Point3<float> Point3<float>::clamp(const Point3<float>& v, float M) {
	return Point3<float>::clamp( v, -FLT_MAX, M );
}

template<>
inline bool Point3<float>::equals( const Point3<float>& p, float epsilon ) const {
	return	fabs( x - p.x ) <= epsilon &&
			fabs( y - p.y ) <= epsilon &&
			fabs( z - p.z ) <= epsilon;
}
