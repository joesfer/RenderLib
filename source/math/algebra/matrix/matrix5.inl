/*
================
Matrix5<T>::Matrix5
================
*/
template< typename T >
Matrix5<T>::Matrix5( void ) {
	memset( m, 0, 5 * 5 * sizeof( T ) );
}

/*
================
Matrix5<T>::Matrix5
================
*/
template< typename T >
Matrix5<T>::Matrix5( const Matrix5<T>& other ) {
	memcpy( (void*)m, (void*)other.m, 5 * 5 * sizeof( T ) );
}

/*
================
Matrix5<T>::Matrix5
================
*/
template< typename T >
Matrix5<T>::Matrix5( T m00, T m01, T m02, T m03, T m04, 
					 T m10, T m11, T m12, T m13, T m14,
					 T m20, T m21, T m22, T m23, T m24,
					 T m30, T m31, T m32, T m33, T m34,
					 T m40, T m41, T m42, T m43, T m44 ) {

	m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03; m[0][4] = m04;
	m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13; m[1][4] = m14;
	m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23; m[2][4] = m24;
	m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33; m[3][4] = m34;
	m[4][0] = m40; m[4][1] = m41; m[4][2] = m42; m[4][3] = m43; m[4][4] = m44;
}

/*
================
Matrix5<T>::determinant
================
*/

template< typename T >
T Matrix5<T>::determinant() const {

	// 2x2 sub-determinants required to calculate 5x5 determinant
	T det2_34_01 = m[3][0] * m[4][1] - m[3][1] * m[4][0];
	T det2_34_02 = m[3][0] * m[4][2] - m[3][2] * m[4][0];
	T det2_34_03 = m[3][0] * m[4][3] - m[3][3] * m[4][0];
	T det2_34_04 = m[3][0] * m[4][4] - m[3][4] * m[4][0];
	T det2_34_12 = m[3][1] * m[4][2] - m[3][2] * m[4][1];
	T det2_34_13 = m[3][1] * m[4][3] - m[3][3] * m[4][1];
	T det2_34_14 = m[3][1] * m[4][4] - m[3][4] * m[4][1];
	T det2_34_23 = m[3][2] * m[4][3] - m[3][3] * m[4][2];
	T det2_34_24 = m[3][2] * m[4][4] - m[3][4] * m[4][2];
	T det2_34_34 = m[3][3] * m[4][4] - m[3][4] * m[4][3];

	// 3x3 sub-determinants required to calculate 5x5 determinant
	T det3_234_012 = m[2][0] * det2_34_12 - m[2][1] * det2_34_02 + m[2][2] * det2_34_01;
	T det3_234_013 = m[2][0] * det2_34_13 - m[2][1] * det2_34_03 + m[2][3] * det2_34_01;
	T det3_234_014 = m[2][0] * det2_34_14 - m[2][1] * det2_34_04 + m[2][4] * det2_34_01;
	T det3_234_023 = m[2][0] * det2_34_23 - m[2][2] * det2_34_03 + m[2][3] * det2_34_02;
	T det3_234_024 = m[2][0] * det2_34_24 - m[2][2] * det2_34_04 + m[2][4] * det2_34_02;
	T det3_234_034 = m[2][0] * det2_34_34 - m[2][3] * det2_34_04 + m[2][4] * det2_34_03;
	T det3_234_123 = m[2][1] * det2_34_23 - m[2][2] * det2_34_13 + m[2][3] * det2_34_12;
	T det3_234_124 = m[2][1] * det2_34_24 - m[2][2] * det2_34_14 + m[2][4] * det2_34_12;
	T det3_234_134 = m[2][1] * det2_34_34 - m[2][3] * det2_34_14 + m[2][4] * det2_34_13;
	T det3_234_234 = m[2][2] * det2_34_34 - m[2][3] * det2_34_24 + m[2][4] * det2_34_23;

	// 4x4 sub-determinants required to calculate 5x5 determinant
	T det4_1234_0123 = m[1][0] * det3_234_123 - m[1][1] * det3_234_023 + m[1][2] * det3_234_013 - m[1][3] * det3_234_012;
	T det4_1234_0124 = m[1][0] * det3_234_124 - m[1][1] * det3_234_024 + m[1][2] * det3_234_014 - m[1][4] * det3_234_012;
	T det4_1234_0134 = m[1][0] * det3_234_134 - m[1][1] * det3_234_034 + m[1][3] * det3_234_014 - m[1][4] * det3_234_013;
	T det4_1234_0234 = m[1][0] * det3_234_234 - m[1][2] * det3_234_034 + m[1][3] * det3_234_024 - m[1][4] * det3_234_023;
	T det4_1234_1234 = m[1][1] * det3_234_234 - m[1][2] * det3_234_134 + m[1][3] * det3_234_124 - m[1][4] * det3_234_123;

	// determinant of 5x5 matrix
	return m[0][0] * det4_1234_1234 - m[0][1] * det4_1234_0234 + m[0][2] * det4_1234_0134 - m[0][3] * det4_1234_0124 + m[0][4] * det4_1234_0123;
}
