#include <math/algebra/quaternion/quaternion.h>

namespace RenderLib {
namespace Math {

Quaternion::Quaternion( float _x, float _y, float _z, float _w) {
	this->x = _x; this->y = _y; this->z = _z; this->w = _w;
}

Quaternion Quaternion::fromAxisRotation(const RenderLib::Math::Vector3f& a, float angle) {
	using namespace RenderLib::Math;
	Vector3f axis = Vector3f::normalize(a);
	float sin_a = sinf( angle / 2 );
	float cos_a = cosf( angle / 2 );
	Quaternion q;
	q.x    = axis.x * sin_a;
	q.y    = axis.y * sin_a;
	q.z    = axis.z * sin_a;
	q.w    = cos_a;
	return q;
}

Quaternion Quaternion::fromEuler(float yaw, float pitch, float roll) {

	float sxcy, cxcy, sxsy, cxsy;

	const float sy(sinf(pitch * 0.5f));
	const float cy(cosf(pitch * 0.5f));
	const float sz(sinf(yaw * 0.5f));
	const float cz(cosf(yaw * 0.5f));
	const float sx(sinf(roll * 0.5f));
	const float cx(cosf(roll * 0.5f));

	sxcy = sx * cy;
	cxcy = cx * cy;
	sxsy = sx * sy;
	cxsy = cx * sy;

	return Quaternion( cxsy*sz - sxcy*cz, -cxsy*cz - sxcy*sz, sxsy*cz - cxcy*sz, cxcy*cz + sxsy*sz );
}

RenderLib::Math::Matrix4f Quaternion::toRotation() const {
	// this quaternion->matrix conversion assumes the quaternion is normalized
	// here's the original code http://cache-www.intel.com/cd/00/00/29/37/293748_293748.pdf
	// We check this in debug at least... FLT_EPSILON is too small for this check, so
	// we're using a larger reasonable epsilon
	assert( fabsf( 1.0f - ( x*x + y*y + z*z + w*w ) ) < 0.003f );

	RenderLib::Math::Matrix4f M;
	float	wx, wy, wz;
	float	xx, yy, yz;
	float	xy, xz, zz;
	float	x2, y2, z2;

	x2 = x + x;
	y2 = y + y;
	z2 = z + z;

	xx = x * x2;
	xy = x * y2;
	xz = x * z2;

	yy = y * y2;
	yz = y * z2;
	zz = z * z2;

	wx = w * x2;
	wy = w * y2;
	wz = w * z2;

	M.m[ 0 ][ 0 ] = 1.0f - ( yy + zz );
	M.m[ 0 ][ 1 ] = xy - wz;
	M.m[ 0 ][ 2 ] = xz + wy;

	M.m[ 1 ][ 0 ] = xy + wz;
	M.m[ 1 ][ 1 ] = 1.0f - ( xx + zz );
	M.m[ 1 ][ 2 ] = yz - wx;

	M.m[ 2 ][ 0 ] = xz - wy;
	M.m[ 2 ][ 1 ] = yz + wx;
	M.m[ 2 ][ 2 ] = 1.0f - ( xx + yy );

	return M;
}

} // namespace Math
} // namespace RenderLib
