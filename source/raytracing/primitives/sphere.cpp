/*
	================================================================================
	This software is released under the LGPL-3.0 license: http://www.opensource.org/licenses/lgpl-3.0.html

	Copyright (c) 2012, Jose Esteve. http://www.joesfer.com

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 3.0 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
	================================================================================
*/

#include <raytracing/primitives/sphere.h>
#include <geometry/utils.h>

namespace RenderLib {
namespace Raytracing {

Sphere Sphere::from3Points(const RenderLib::Math::Point3f& a, const RenderLib::Math::Point3f& b, const RenderLib::Math::Point3f& c) {
	Sphere s;
	RenderLib::Geometry::sphereFrom3Points(a, b, c, s.center, s.radius);
    return s;
}

// Realtime Collision Detection. page 178
bool Sphere::intersection(const RenderLib::Raytracing::Ray& r, float& isectT ) const {
	using namespace RenderLib::Math;
	Vector3f m = r.origin - center;
	float b = Vector3f::dot(m, r.direction);
    float c = Vector3f::dot(m, m) - radius * radius;
    // Exit if the ray origin is outside the sphere and the direction is pointing away from it
    if ( c > 0.0f && b > 0.0f ) return false;
    float discr = b*b - c;
    // A negative discriminant corresponds to a ray missing sphere
    if ( discr < 0.0f ) return false;
    // Ray now found to intersect sphere, compute smallest t value of intersection
    isectT = -b - sqrtf(discr);
    // if t is negative, ray started inside sphere, so clamp to zero
    if ( isectT < 0.0f ) isectT = 0.0f;
    return true;
}

} // namespace Raytracing
} // namespace RenderLib