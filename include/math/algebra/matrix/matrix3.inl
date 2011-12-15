/*
================
Matrix3<T>::Matrix3
================
*/
template< typename T >
Matrix3<T>::Matrix3( void ) {
	memset( m, 0, 3 * 3 * sizeof( T ) );
}

/*
================
Matrix3<T>::Matrix3
================
*/
template< typename T >
Matrix3<T>::Matrix3( const Matrix3<T>& other ) {
	memcpy( (void*)m, (void*)other.m, 3 * 3 * sizeof( T ) );
}

/*
================
Matrix3<T>::Matrix3
================
*/
template< typename T >
Matrix3<T>::Matrix3( T m00, T m01, T m02,
				T m10, T m11, T m12,
				T m20, T m21, T m22 ) {

	m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
	m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
	m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; 
}
/*
================
Matrix3<T>::transposed
================
*/
template< typename T >
Matrix3<T> Matrix3<T>::transposed() const {
	return Matrix3<T>(	m[0][0], m[1][0], m[2][0],
						m[0][1], m[1][1], m[2][1],
						m[0][2], m[1][2], m[2][2] );
}

/*
================
Matrix3<T>::transpose
================
*/
template< typename T >
Matrix3<T>& Matrix3<T>::transpose() {
	T tmp0, tmp1, tmp2;

	tmp0 = m[0][1];
	m[0][1] = m[1][0];
	m[1][0] = tmp0;
	tmp1 = m[0][2];
	m[0][2] = m[2][0];
	m[2][0] = tmp1;
	tmp2 = m[1][2];
	m[1][2] = m[2][1];
	m[2][1] = tmp2;

	return *this;
}

/*
================
Matrix3<T>::determinant
================
*/
template< typename T >
T Matrix3<T>::determinant() const {

	T det2_12_01 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
	T det2_12_02 = m[1][0] * m[2][2] - m[1][2] * m[2][0];
	T det2_12_12 = m[1][1] * m[2][2] - m[1][2] * m[2][1];

	return m[0][0] * det2_12_12 - m[0][1] * det2_12_02 + m[0][2] * det2_12_01;
}