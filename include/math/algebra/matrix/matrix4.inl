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
			transpose.m[ i ][ j ] = m[ j ][ i ];
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


/*
================
Matrix4<T>::transform
================
*/
template< typename T >
Vector3<T>  Matrix4<T>::transform(const Vector3<T>& v) const {
	// w = 0
	return Vector3<T>(	m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
						m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
						m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z );
}


/*
================
Matrix4<T>::transform
================
*/
template< typename T >
Point3<T>  Matrix4<T>::transform(const Point3<T>& p) const {
	// w = 1
	float x = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
	float y = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
	float z = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];
	float w = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3];
	return Point3<T>( x / w, y / w, z / w );
}

/*
================
Matrix4<T>::operator *
================
*/
template< typename T >
Matrix4<T> Matrix4<T>::operator * (const Matrix4<T>& M ) const {
	Matrix4<T> dst;

	for ( int i = 0; i < 4; i++ ) {
		for ( int j = 0; j < 4; j++ ) {
			dst.m[i][j] = 0;
			for( int k = 0; k < 4; k++ ) {
				dst.m[i][j] += m[i][k] * M.m[k][j];
			}
		}
	}
	return dst;
}

/*
================
Matrix4<T>::rotationFromAngles
================
*/
template< typename T >
Matrix4<T> Matrix4<T>::rotationFromAngles(float yaw, float pitch, float roll) {
	const float phi = yaw;
	const float theta = pitch;
	const float psi = roll;

	float cosPhi = cosf(phi);
	float sinPhi = sinf(phi);
	float cosTheta = cosf(theta);
	float sinTheta = sinf(theta);
	float cosPsi = cosf(psi);
	float sinPsi = sinf(psi);

	return Matrix4( cosTheta*cosPsi, -cosPhi*sinPsi + sinPhi*sinTheta*cosPsi,  sinPhi*sinPsi + cosPhi*sinTheta*cosPsi, 0,
		cosTheta*sinPsi,  cosPhi*cosPsi + sinPhi*sinTheta*sinPsi, -sinPhi*cosPsi + cosPhi*sinTheta*sinPsi, 0,
		-sinTheta,         sinPhi*cosTheta,                           cosPhi*cosTheta,                     0,
		0,                 0,                                         0,                                   1);
}

/*
================
Matrix4<T>::toAngles
================
*/
template< typename T >
void Matrix4<T>::toAngles(float& yaw, float& pitch, float& roll) const {
	float		theta;
	float		cp;
	float		sp;

	sp = m[ 2 ][ 0 ];

	// cap off our sin value so that we don't get any NANs
	if ( sp > 1.0f ) {
		sp = 1.0f;
	} else if ( sp < -1.0f ) {
		sp = -1.0f;
	}

	theta = -asinf( sp );
	cp = cosf( theta );

	if ( cp > 8192.0f * FLT_EPSILON ) {
		pitch	= theta;
		roll	= atan2f( m[ 1 ][ 0 ], m[ 0 ][ 0 ] );
		yaw	= atan2f( m[ 2 ][ 1 ], m[ 2 ][ 2 ] );
	} else {
		pitch	= theta;
		roll	= -atan2f( m[ 0 ][ 1 ], m[ 1 ][ 1 ] );
		yaw	= 0.0f;
	}
}

/*
================
Matrix4<T>::inverse
================
*/
template< typename T >
Matrix4<T> Matrix4<T>::inverse() const {
	Matrix4<T> inv = *this;
	return inv.invert();
}

/*
================
Matrix4<T>::invert
================
*/
template< typename T >
Matrix4<T>& Matrix4<T>::invert() {
	T inv[4][4], det;

	inv[0][0] =   m[1][1]*m[2][2]*m[3][3] - m[1][1]*m[2][3]*m[3][2] - m[2][1]*m[1][2]*m[3][3] + m[2][1]*m[1][3]*m[3][2] + m[3][1]*m[1][2]*m[2][3] - m[3][1]*m[1][3]*m[2][2];
	inv[1][0] =  -m[1][0]*m[2][2]*m[3][3] + m[1][0]*m[2][3]*m[3][2] + m[2][0]*m[1][2]*m[3][3] - m[2][0]*m[1][3]*m[3][2] - m[3][0]*m[1][2]*m[2][3] + m[3][0]*m[1][3]*m[2][2];
	inv[2][0] =   m[1][0]*m[2][1]*m[3][3] - m[1][0]*m[2][3]*m[3][1] - m[2][0]*m[1][1]*m[3][3] + m[2][0]*m[1][3]*m[3][1] + m[3][0]*m[1][1]*m[2][3] - m[3][0]*m[1][3]*m[2][1];
	inv[3][0] = -m[1][0]*m[2][1]*m[3][2] + m[1][0]*m[2][2]*m[3][1] + m[2][0]*m[1][1]*m[3][2] - m[2][0]*m[1][2]*m[3][1] - m[3][0]*m[1][1]*m[2][2] + m[3][0]*m[1][2]*m[2][1];
	inv[0][1] =  -m[0][1]*m[2][2]*m[3][3] + m[0][1]*m[2][3]*m[3][2] + m[2][1]*m[0][2]*m[3][3] - m[2][1]*m[0][3]*m[3][2] - m[3][1]*m[0][2]*m[2][3] + m[3][1]*m[0][3]*m[2][2];
	inv[1][1] =   m[0][0]*m[2][2]*m[3][3] - m[0][0]*m[2][3]*m[3][2] - m[2][0]*m[0][2]*m[3][3] + m[2][0]*m[0][3]*m[3][2] + m[3][0]*m[0][2]*m[2][3] - m[3][0]*m[0][3]*m[2][2];
	inv[2][1] =  -m[0][0]*m[2][1]*m[3][3] + m[0][0]*m[2][3]*m[3][1] + m[2][0]*m[0][1]*m[3][3] - m[2][0]*m[0][3]*m[3][1] - m[3][0]*m[0][1]*m[2][3] + m[3][0]*m[0][3]*m[2][1];
	inv[3][1] =  m[0][0]*m[2][1]*m[3][2] - m[0][0]*m[2][2]*m[3][1] - m[2][0]*m[0][1]*m[3][2] + m[2][0]*m[0][2]*m[3][1] + m[3][0]*m[0][1]*m[2][2] - m[3][0]*m[0][2]*m[2][1];
	inv[0][2] =   m[0][1]*m[1][2]*m[3][3] - m[0][1]*m[1][3]*m[3][2] - m[1][1]*m[0][2]*m[3][3] + m[1][1]*m[0][3]*m[3][2] + m[3][1]*m[0][2]*m[1][3] - m[3][1]*m[0][3]*m[1][2];
	inv[1][2] =  -m[0][0]*m[1][2]*m[3][3] + m[0][0]*m[1][3]*m[3][2] + m[1][0]*m[0][2]*m[3][3] - m[1][0]*m[0][3]*m[3][2] - m[3][0]*m[0][2]*m[1][3] + m[3][0]*m[0][3]*m[1][2];
	inv[2][2] =  m[0][0]*m[1][1]*m[3][3] - m[0][0]*m[1][3]*m[3][1] - m[1][0]*m[0][1]*m[3][3] + m[1][0]*m[0][3]*m[3][1] + m[3][0]*m[0][1]*m[1][3] - m[3][0]*m[0][3]*m[1][1];
	inv[3][2] = -m[0][0]*m[1][1]*m[3][2] + m[0][0]*m[1][2]*m[3][1] + m[1][0]*m[0][1]*m[3][2] - m[1][0]*m[0][2]*m[3][1] - m[3][0]*m[0][1]*m[1][2] + m[3][0]*m[0][2]*m[1][1];
	inv[0][3] =  -m[0][1]*m[1][2]*m[2][3] + m[0][1]*m[1][3]*m[2][2] + m[1][1]*m[0][2]*m[2][3] - m[1][1]*m[0][3]*m[2][2] - m[2][1]*m[0][2]*m[1][3] + m[2][1]*m[0][3]*m[1][2];
	inv[1][3] =   m[0][0]*m[1][2]*m[2][3] - m[0][0]*m[1][3]*m[2][2] - m[1][0]*m[0][2]*m[2][3] + m[1][0]*m[0][3]*m[2][2] + m[2][0]*m[0][2]*m[1][3] - m[2][0]*m[0][3]*m[1][2];
	inv[2][3] = -m[0][0]*m[1][1]*m[2][3] + m[0][0]*m[1][3]*m[2][1] + m[1][0]*m[0][1]*m[2][3] - m[1][0]*m[0][3]*m[2][1] - m[2][0]*m[0][1]*m[1][3] + m[2][0]*m[0][3]*m[1][1];
	inv[3][3] =  m[0][0]*m[1][1]*m[2][2] - m[0][0]*m[1][2]*m[2][1] - m[1][0]*m[0][1]*m[2][2] + m[1][0]*m[0][2]*m[2][1] + m[2][0]*m[0][1]*m[1][2] - m[2][0]*m[0][2]*m[1][1];

	det = m[0][0]*inv[0][0] + m[0][1]*inv[1][0] + m[0][2]*inv[2][0] + m[0][3]*inv[3][0];
	if (det == 0) {
		assert(false);
	} else {
		det = 1.0f / det;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				m[i][j] = inv[i][j] * det;
			}
		}
	}

	return *this;
}
