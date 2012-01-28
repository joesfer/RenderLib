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
#include <math/algebra/vector/vector3.h>

namespace RenderLib {
namespace Geometry {

	class BoundingBox {
	public:
		BoundingBox();
		BoundingBox(const ::RenderLib::Math::Point3f &p);
		BoundingBox(const ::RenderLib::Math::Point3f &p1, const ::RenderLib::Math::Point3f &p2);
		
		const ::RenderLib::Math::Point3f& max() const;
		const ::RenderLib::Math::Point3f& min() const;
		::RenderLib::Math::Point3f& max();
		::RenderLib::Math::Point3f& min();

		float						volume() const;
		::RenderLib::Math::Vector3f	extents() const;
		int							shortestAxis() const;
		int							longestAxis() const;
		float						surfaceArea() const;
		::RenderLib::Math::Point3f	center() const;

		bool overlaps(const BoundingBox &b) const;
		bool inside(const ::RenderLib::Math::Point3f &p) const;

		void expand( const ::RenderLib::Math::Point3f& p );
		void expand( const BoundingBox& b );
		void expand(float delta);

		static BoundingBox getUnion(const BoundingBox& b1, const BoundingBox& b2);
		void boundingSphere(RenderLib::Math::Point3f& c, float& rad) const;
		

		private:
			RenderLib::Math::Point3f pMin, pMax;
	};
}
}