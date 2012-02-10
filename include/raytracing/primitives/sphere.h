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

#pragma once

#include <math/algebra/point/point3.h>
#include <raytracing/ray/ray.h>

namespace RenderLib {
namespace Raytracing {
	
class Sphere {
public:
    Sphere() : radius(0) {}
	Sphere( const RenderLib::Math::Point3f& c, float r ) : center(c), radius(r) {}
	static Sphere from3Points(const RenderLib::Math::Point3f& a, const RenderLib::Math::Point3f& b, const RenderLib::Math::Point3f& c);

	bool intersection(const RenderLib::Raytracing::Ray& r, float& isectT ) const;

    RenderLib::Math::Point3f center;
    float radius;
};

} // namespace Raytracing
} // namespace RenderLib