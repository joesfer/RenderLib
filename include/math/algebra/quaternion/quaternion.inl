
Quaternion Quaternion::operator*(const Quaternion& Q) const {
	Quaternion res;
	res.w = w * Q.w - x * Q.x - y * Q.y - z * Q.z;
	res.x = w * Q.x + x * Q.w + y * Q.z - z * Q.y;
	res.y = w * Q.y + y * Q.w + z * Q.x - x * Q.z;
	res.z = w * Q.z + z * Q.w + x * Q.y - y * Q.x;
	return res;
}

Quaternion& Quaternion::normalize() {
	float magnitude = sqrtf(w*w + x*x + y*y + z*z);
	if ( magnitude > 1e-5f ) {
		float invMag =  1.0f / magnitude;
		x *= invMag;
		y *= invMag;
		z *= invMag;
		w *= invMag;
	}
	return *this;
}

Quaternion Quaternion::conjugate() const {
	Quaternion q;
	q.x = -x;
	q.y = -y;
	q.z = -z;
	q.w = w;
	return q;
}