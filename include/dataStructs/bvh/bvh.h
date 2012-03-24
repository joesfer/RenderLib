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

#include <vector>
#include <float.h>
#include <math/algebra/vector/vector3.h>
#include <raytracing/ray/ray.h>
#include <raytracing/primitives/sphere.h>
#include <geometry/bounds/boundingBox.h>

namespace RenderLib {
namespace DataStructures {

class BVH {
public:
    BVH( const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices );
    bool intersection(const RenderLib::Raytracing::Ray& r,
                      const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices,
                      RenderLib::Math::Vector3f& hitpoint,
					  int* triangleIndex = NULL,
					  float* barycentricU = NULL, 
					  float* barycentricV = NULL ) const;
private:

    class BVHNode {
    public:
        bool leaf;
        int volumeIndex; // index in volumes
        union {
            int primitiveIndex; // for leaves
            int leftChildIndex; // index in nodes array: rightChild = leftChild + 1
        };
    };

    void split( size_t nodeIndex,
                const std::vector<RenderLib::Math::Vector3f>& vertices,
                const std::vector<int>& indices,
                const std::vector<RenderLib::Math::Vector3f>& centroids,
                std::vector<int>& primitives,
                const RenderLib::Geometry::BoundingBox& bounds );

	struct hit_t;
	RenderLib::Raytracing::Sphere boundingSphere( const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices ) const;
    bool intersection_r(const RenderLib::Raytracing::Ray& r,
                        const std::vector<RenderLib::Math::Vector3f>& vertices,
                        const std::vector<int>& indices,
                        int node,
						hit_t& hit ) const;

    std::vector<BVHNode> nodes;
    std::vector<RenderLib::Raytracing::Sphere> volumes;
};

} // namespace DataStructures
} // namespace RenderLib