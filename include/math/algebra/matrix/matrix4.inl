/*
================
Matrix4<T>::Matrix4
================
*/
template< typename T >
Matrix4<T>::Matrix4( void ) {
	memset( m, 0, 4 * 4 * sizeof( T ) );
}

/*
================
Matrix4<T>::Matrix4
================
*/
template< typename T >
Matrix4<T>::Matrix4( const Matrix4<T>& other ) {
	memcpy( (void*)m, (void*)other.m, 4 * 4 * sizeof( T ) );
}

/*
================
Matrix4<T>::Matrix4
================
*/
template< typename T >
Matrix4<T>::Matrix4( T m00, T m01, T m02, T m03,
					 T m10, T m11, T m12, T m13,
					 T m20, T m21, T m22, T m23,
					 T m30, T m31, T m32, T m33 ) {

	 m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03; 
	 m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13; 
	 m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23; 
	 m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33; 
}


/*
================
Matrix4<T>::transposed
================
*/
template< typename T >
Matrix4<T> Matrix4<T>::transposed() const {
	Matrix4<T>	transpose;
	int		i, j;

	for( i = 0; i < 4; i++ ) {
		for( j = 0; j < 4; j++ ) {
			transpose[ i ][ j ] = m[ j ][ i ];
		}
	}
	return transpose;
}

/*
================
Matrix4<T>::transpose
================
*/
template< typename T >
Matrix4<T>& Matrix4<T>::transpose() {
	T	temp;
	int	i, j;

	for( i = 0; i < 4; i++ ) {
		for( j = i + 1; j < 4; j++ ) {
			temp = m[ i ][ j ];
			m[ i ][ j ] = m[ j ][ i ];
			m[ j ][ i ] = temp;
		}
	}
	return *this;
}

/*
================
Matrix4<T>::determinant
================
*/
template< typename T >
T Matrix4<T>::determinant() const {

	// 2x2 sub-determinants
	T det2_01_01 = m[0][0] * m[1][1] - m[0][1] * m[1][0];
	T det2_01_02 = m[0][0] * m[1][2] - m[0][2] * m[1][0];
	T det2_01_03 = m[0][0] * m[1][3] - m[0][3] * m[1][0];
	T det2_01_12 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
	T det2_01_13 = m[0][1] * m[1][3] - m[0][3] * m[1][1];
	T det2_01_23 = m[0][2] * m[1][3] - m[0][3] * m[1][2];

	// 3x3 sub-determinants
	T det3_201_012 = m[2][0] * det2_01_12 - m[2][1] * det2_01_02 + m[2][2] * det2_01_01;
	T det3_201_013 = m[2][0] * det2_01_13 - m[2][1] * det2_01_03 + m[2][3] * det2_01_01;
	T det3_201_023 = m[2][0] * det2_01_23 - m[2][2] * det2_01_03 + m[2][3] * det2_01_02;
	T det3_201_123 = m[2][1] * det2_01_23 - m[2][2] * det2_01_13 + m[2][3] * det2_01_12;

	return ( - det3_201_123 * m[3][0] + det3_201_023 * m[3][1] - det3_201_013 * m[3][2] + det3_201_012 * m[3][3] );
}