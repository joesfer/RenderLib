#pragma once

#include <math.h>
#include <math/algebra/matrix/matrix4.h>
#include <math/algebra/vector/vector3.h>

namespace RenderLib {
namespace Math {

//////////////////////////////////////////////////////////////////////////////////////////
// Quaternion
//////////////////////////////////////////////////////////////////////////////////////////

class Quaternion {
public:
	inline Quaternion( float x, float y, float z, float w);
	static Quaternion fromAxisRotation(const RenderLib::Math::Vector3f& axis, float angle);
	static Quaternion fromEuler( float yaw, float pitch, float roll );

	RenderLib::Math::Matrix4f toRotation() const;
	inline Quaternion operator*(const Quaternion& Q) const;
	inline Quaternion& normalize();
	inline Quaternion conjugate() const;
public:
	float x,y,z,w;
private:
	explicit Quaternion() {}
};

#include "quaternion.inl"

} // namespace Math
} // namespace RenderLib
