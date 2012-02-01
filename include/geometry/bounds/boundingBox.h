#pragma once

#include <math/algebra/point/point3.h>
#include <math/algebra/vector/vector3.h>

namespace RenderLib {
namespace Geometry {

	class BoundingBox {
	public:
		BoundingBox();
		BoundingBox(const RenderLib::Math::Point3f &p);
		BoundingBox(const RenderLib::Math::Point3f &p1, const RenderLib::Math::Point3f &p2);
		
		const RenderLib::Math::Point3f& max() const;
		const RenderLib::Math::Point3f& min() const;
		RenderLib::Math::Point3f& max();
		RenderLib::Math::Point3f& min();

		float						volume() const;
		RenderLib::Math::Vector3f	extents() const;
		int							shortestAxis() const;
		int							longestAxis() const;
		float						surfaceArea() const;
		RenderLib::Math::Point3f	center() const;

		bool overlaps(const BoundingBox &b) const;
		bool inside(const RenderLib::Math::Point3f &p) const;

		void expand( const RenderLib::Math::Point3f& p );
		void expand( const RenderLib::Math::Point3d& p );
		void expand( const BoundingBox& b );
		void expand(float delta);

		static BoundingBox getUnion(const BoundingBox& b1, const BoundingBox& b2);
		void boundingSphere(RenderLib::Math::Point3f& c, float& rad) const;
		

		private:
			RenderLib::Math::Point3f pMin, pMax;
	};
}
}