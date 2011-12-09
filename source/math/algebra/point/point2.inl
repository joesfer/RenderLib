
template< class T >
inline Point2<T>  Point2<T>::operator + (const Point2<T>& p) const { 
	return Point2<T>(x + p.x, y + p.y ); 
}

template< class T >
inline Point2<T>  Point2<T>::operator + (const Vector2<T>& v) const { 
	return Point2<T>(x + v.x, y + v.y ); 
}

template< class T >
inline Vector2<T> Point2<T>::operator - (const Point2<T>& p) const { 
	return Vector2<T>(x - p.x, y - p.y ); // that's how we usually define a vector, from two points
}

template< class T >
inline bool Point2<T>::operator == (const Point2<T>& p) const { 
	return x == p.x && y == p.y; 
}

template< class T >
inline bool Point2<T>::operator != (const Point2<T>& p) const { 
	return x != p.x || y != p.y; 
}

template< class T >
inline Point2<T> Point2<T>::operator * (T f) const { 
	return Point2<T>( x*f, y*f ); 
}

template< class T >
inline Point2<T> Point2<T>::operator / (T f) const { 
	assert(f != 0); return (*this) * ( 1.0f / f ); 
}

template< class T >
inline void Point2<T>::operator -= (const Vector2<T>& v) { 
	x -= v.x; 
	y -= v.y; 
}

template< class T >
inline void Point2<T>::operator -= (const Point2<T>& p) { 
	x -= p.x; 
	y -= p.y; 
}

template< class T >
inline void Point2<T>::operator += (const Vector2<T>& v) { 
	x += v.x; 
	y += v.y; 
}

template< class T >
inline void Point2<T>::operator += (const Point2<T>& p) { 
	x += p.x; 
	y += p.y; 
}

template< class T >
inline void Point2<T>::operator *= (const T a)	{ 
	x *= a; 
	y *= a; 
}

template< class T >
inline void Point2<T>::operator /= (const T a) { 
	x /= a; 
	y /= a; 
}

template< class T >
inline Point2<T> Point2<T>::operator -() const {
	return Point2<T>( -x, -y );
}

template< class T >
T Point2<T>::operator[](int i) const { 
	assert(i >= 0 && i <= 1);
	return (&x)[i]; 
}	

template< class T >
inline T& Point2<T>::operator[] (const int i) {
	assert(i >= 0 && i <= 1);
	return (&x)[i];
}

template< class T >
inline T Point2<T>::distance(const Point2<T>& p1, const Point2<T>& p2) { 
	return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)); 
}

template< class T >
inline T Point2<T>::distanceSquared(const Point2<T>& p1, const Point2<T>& p2) { 
	return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)); 
}

template< class T >
inline T Point2<T>::distanceTo( const Point2<T>& p2 ) const { 
	return distance( *this, p2 ); 
}

template< class T >
inline T Point2<T>::distanceToOrigin() const { 
	return distance( *this, Point2<T>() ); 
}

template< class T >
inline T Point2<T>::distanceSquaredTo( const Point2<T>& p2 ) const { 
	return distanceSquared( *this, p2 ); }

template< class T >
inline T Point2<T>::distanceSquaredToOrigin() const { 
	return distanceSquared( *this, Point2<T>() ); 
}

template< class T >
inline Point2<T> Point2<T>::clamp(const Point2<T>& v, T m, T M) {
	return Point2<T>( std::min( M, std::max( m, v.x ) ), std::min( M, std::max( m, v.y ) ) );
}

template< class T >
inline Point2<T> Point2<T>::clamp(const Point2<T>& v, const Point2<T>& m, const Point2<T>& M ) {
	return Point2<T>( std::min( M.x, std::max( m.x, v.x ) ), std::min( M.y, std::max( m.y, v.y ) ) );
}

template< class T >
inline bool Point2<T>::equals( const Point2<T>& p, T epsilon ) const {
	return	abs( x - p.x ) <= epsilon &&
			abs( y - p.y ) <= epsilon;
}

// specializations for Point2d ////////////////////////////////////////////////////

template<>
inline Point2<double> Point2<double>::clamp(const Point2<double>& v, double M) {
	return Point2<double>::clamp( v, -DBL_MAX, M );
}

// specializations for Point2f ////////////////////////////////////////////////////

template<> 
inline float Point2<float>::distance( const Point2<float>& p1, const Point2<float>& p2 ) { 
	return sqrtf((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)); 
}	

template<>
inline Point2<float> Point2<float>::clamp(const Point2<float>& v, float M) {
	return Point2<float>::Clamp( v, -FLT_MAX, M );
}

template<>
inline bool Point2<float>::equals( const Point2<float>& p, float epsilon ) const {
	return	fabs( x - p.x ) <= epsilon &&
			fabs( y - p.y ) <= epsilon;
}
