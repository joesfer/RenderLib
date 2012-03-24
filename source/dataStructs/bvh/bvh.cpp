#include <assert.h>
#include <math.h>
#include <iostream>
#include <math/algebra/matrix/matrix3.h>
#include <geometry/intersection/intersection.h>
#include <dataStructs/bvh/bvh.h>

namespace RenderLib {
namespace DataStructures {

BVH::BVH(const std::vector<RenderLib::Math::Vector3f> &vertices, const std::vector<int> &indices) {
    using namespace std;
	using namespace RenderLib::Math;
	using namespace RenderLib::Raytracing;
	using namespace RenderLib::Geometry;
    assert( indices.size() >= 3 );

    vector<Vector3f> centroids;
    const size_t numPrimitives = indices.size() / 3;
    if ( numPrimitives == 0 ) return;

    centroids.reserve( numPrimitives );
    BoundingBox bounds;
    for( size_t i = 0; i < indices.size(); i += 3 ) {
        const Vector3f& a = vertices[indices[i]];
        const Vector3f& b = vertices[indices[i + 1]];
        const Vector3f& c = vertices[indices[i + 2]];
        bounds.expand(a);
        bounds.expand(b);
        bounds.expand(c);
        centroids.push_back( ( a + b + c ) / 3 );
    }

    // calculate sphere containing all points
    Sphere rootSphere;
    rootSphere.center = bounds.center();
    rootSphere.radius = (bounds.center() - bounds.min()).length();
    BVHNode root;
    root.volumeIndex = 0;
    volumes.push_back( rootSphere );
    nodes.push_back(root);

    if ( numPrimitives == 1 ) {
        root.leaf = true;
        root.primitiveIndex = 0;
    } else {
        vector<int> primitives;
        primitives.reserve(numPrimitives);
        for( size_t i = 0; i < numPrimitives; i++ ) primitives.push_back((int)i);
        split(0, vertices, indices, centroids, primitives, bounds);
    }
}

void BVH::split( size_t nodeIndex,
                 const std::vector<RenderLib::Math::Vector3f>& vertices,
                 const std::vector<int>& indices,
                 const std::vector<RenderLib::Math::Vector3f>& centroids,
                 std::vector<int>& primitives,
				 const RenderLib::Geometry::BoundingBox& bounds ) {
    using namespace std;
	using namespace RenderLib::Math;
	using namespace RenderLib::Raytracing;
	using namespace RenderLib::Geometry;
    assert( primitives.size() > 0 );
    nodes.reserve( nodes.size() + 2 ); // alloc enough space to prevent vector resizing within this method
    BVH::BVHNode& node = nodes[ nodeIndex ];
    if ( primitives.size() == 1 ) {
        node.leaf = true;
        node.primitiveIndex = primitives[0];
        // tighten volume bounds
        volumes[node.volumeIndex] = Sphere::from3Points( vertices[indices[3*node.primitiveIndex]],
                                                             vertices[indices[3*node.primitiveIndex + 1]],
                                                             vertices[indices[3*node.primitiveIndex + 2]] );
    } else {
        node.leaf = false;
        vector<int> left;
        left.reserve(primitives.size() / 2 * 3 );
        vector<int> right;
        right.reserve(primitives.size() / 2 * 3 );

        int longestAxis = bounds.longestAxis();
        float longestAxisLength = bounds.extents()[longestAxis];
        assert(longestAxisLength > 0);

        // classify primitives with regards of the spatial mean

        float cMin = FLT_MAX;
        float cMax = -FLT_MAX;
        for( size_t i = 0; i < primitives.size(); i++ ) {
            int p = primitives[i];
            const Vector3f& c = centroids[p];
            cMin = std::min( cMin, c[longestAxis] );
            cMax = std::max( cMax, c[longestAxis] );
        }
        float mean = ( cMin + cMax ) * 0.5f;

        BoundingBox leftBounds, rightBounds;
        for( size_t i = 0; i < primitives.size(); i++ ) {
            int p = primitives[i];
            const Vector3f& c = centroids[p];
            if ( c[longestAxis] <= mean ) {
                // assign to left
                leftBounds.expand( vertices[indices[3 * p + 0]]);
                leftBounds.expand( vertices[indices[3 * p + 1]]);
                leftBounds.expand( vertices[indices[3 * p + 2]]);
                left.push_back(p);
            } else {
                // assign to right
                rightBounds.expand( vertices[indices[3 * p + 0]]);
                rightBounds.expand( vertices[indices[3 * p + 1]]);
                rightBounds.expand( vertices[indices[3 * p + 2]]);
                right.push_back(p);
            }
        }
        // free no longer used primitives array
        primitives.clear();

        size_t leftChildIndex, rightChildIndex;
        {
            // create left child
            BVHNode leftChild;
            Sphere leftSphere( leftBounds.center(), (leftBounds.center() - leftBounds.min()).length() );
            leftChild.volumeIndex = (int)volumes.size();
            volumes.push_back(leftSphere);

            // create right child
            BVHNode rightChild;
            Sphere rightSphere( rightBounds.center(), (rightBounds.center() - rightBounds.min()).length() );
            rightChild.volumeIndex = (int)volumes.size();
            volumes.push_back(rightSphere);

            node.leftChildIndex = (int)nodes.size();
            leftChildIndex = node.leftChildIndex;
            rightChildIndex = leftChildIndex + 1;
            nodes.push_back(leftChild);
            nodes.push_back(rightChild);
        }

        split( leftChildIndex, vertices, indices, centroids, left, leftBounds );
        split( rightChildIndex, vertices, indices, centroids, right, rightBounds );
    }
}

RenderLib::Raytracing::Sphere BVH::boundingSphere( const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices ) const {
	using namespace RenderLib::Math;
	using namespace RenderLib::Raytracing;
	using namespace RenderLib::Geometry;

	BoundingBox bounds;
	for( size_t i = 0; i < indices.size(); i += 3 ) {
		const Vector3f& a = vertices[indices[i]];
		const Vector3f& b = vertices[indices[i + 1]];
		const Vector3f& c = vertices[indices[i + 2]];
		bounds.expand(a);
		bounds.expand(b);
		bounds.expand(c);
	}
	Sphere s;
	s.center = bounds.center();
	s.radius = (bounds.center() - bounds.min()).length();
	return s;
}

struct BVH::hit_t {
	int a, b, c; // vertex index
	int triangle; // triangle index
	float t; // segment dist
	float v,w; // barycentric coords
};

bool BVH::intersection(const RenderLib::Raytracing::Ray &r,
						const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices,
						RenderLib::Math::Vector3f& isect,
						int* triangleIndex,
						float* barycentricU, 
						float* barycentricV ) const {
	hit_t hit;
	bool res = intersection_r(r, vertices, indices, 0, hit );
	if ( res ) {
		isect = vertices[hit.a] + (vertices[hit.b] -  vertices[hit.a] ) * hit.v + ( vertices[hit.c] - vertices[hit.a] ) * hit.w;
		if ( triangleIndex != NULL ) *triangleIndex = hit.triangle;
		if ( barycentricU != NULL ) *barycentricU = hit.v;
		if ( barycentricV != NULL ) *barycentricV = hit.w;
	}
	return res;
}

bool BVH::intersection_r(const RenderLib::Raytracing::Ray& r,
                         const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices,
                         int node,
                         hit_t& hit ) const {
	using namespace RenderLib::Math;
	using namespace RenderLib::Raytracing;
	using namespace RenderLib::Geometry;

    const BVHNode& n = nodes[node];
    float t;
    if( !volumes[n.volumeIndex].intersection(r, t)) return false;

    // got intersection
    if( n.leaf ) {
        // we have an intersection with a leaf volume, refine with the actual primitive.
        hit.a = indices[ 3 * n.primitiveIndex ];
        hit.b = indices[ 3 * n.primitiveIndex + 1 ];
        hit.c = indices[ 3 * n.primitiveIndex + 2 ];
		hit.triangle = n.primitiveIndex;
        const Sphere& volume = volumes[ n.volumeIndex ];
        const Point3f q = r.origin + r.direction * ((r.origin - volume.center).length() + volume.radius);
		return segmentTriangleIntersect_SingleSided<float>( r.origin, q, vertices[hit.a], vertices[hit.b], vertices[hit.c], hit.t, hit.v, hit.w);
		
    } else {
        // sort children by distance to ray origin
        // and test in that order (hoping the closest one
        // will intersect first)
        const int leftChildIdx = n.leftChildIndex;
        const int rightChildIdx = leftChildIdx + 1;
        const Sphere& leftChildVolume = volumes[ nodes[leftChildIdx].volumeIndex ];
        const Sphere& rightChildVolume = volumes[ nodes[rightChildIdx].volumeIndex ];
        float distLeft = Vector3f::dot(r.direction, leftChildVolume.center - r.origin);
        float distRight = Vector3f::dot(r.direction, rightChildVolume.center - r.origin);
        if ( distLeft <= distRight ) {
            // recurse first left, then right
            if (intersection_r(r, vertices, indices, leftChildIdx, hit)) return true;
            return intersection_r(r, vertices, indices, rightChildIdx, hit);
        } else {
            // recurse first right, then left
            if (intersection_r(r, vertices, indices, rightChildIdx, hit)) return true;
            return intersection_r(r, vertices, indices, leftChildIdx, hit);
        }
    }
}

} // namespace DataStructures
} // namespace RenderLib