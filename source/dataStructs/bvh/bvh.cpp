#include <assert.h>
#include <math.h>
#include <iostream>
#include <math/algebra/matrix/matrix3.h>
#include <dataStructs/bvh/bvh.h>

namespace RenderLib {
namespace DataStructures {

bool segmentTriIntersection( const RenderLib::Math::Point3f& p, const RenderLib::Math::Point3f& q,
                            const RenderLib::Math::Point3f& a, const RenderLib::Math::Point3f& b, const RenderLib::Math::Point3f& c,
                            RenderLib::Math::Vector3f& isect);

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
    rootSphere.m_center = bounds.center();
    rootSphere.m_radius = (bounds.center() - bounds.min()).length();
    BVHNode root;
    root.m_volumeIndex = 0;
    m_volumes.push_back( rootSphere );
    m_nodes.push_back(root);

    if ( numPrimitives == 1 ) {
        root.m_leaf = true;
        root.m_primitiveIndex = 0;
    } else {
        vector<int> primitives;
        primitives.reserve(numPrimitives);
        for( size_t i = 0; i < numPrimitives; i++ ) primitives.push_back(i);
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
    m_nodes.reserve( m_nodes.size() + 2 ); // alloc enough space to prevent vector resizing within this method
    BVH::BVHNode& node = m_nodes[ nodeIndex ];
    if ( primitives.size() == 1 ) {
        node.m_leaf = true;
        node.m_primitiveIndex = primitives[0];
        // tighten volume bounds
        m_volumes[node.m_volumeIndex] = Sphere::from3Points( vertices[indices[3*node.m_primitiveIndex]],
                                                             vertices[indices[3*node.m_primitiveIndex + 1]],
                                                             vertices[indices[3*node.m_primitiveIndex + 2]] );
    } else {
        node.m_leaf = false;
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
            leftChild.m_volumeIndex = m_volumes.size();
            m_volumes.push_back(leftSphere);

            // create right child
            BVHNode rightChild;
            Sphere rightSphere( rightBounds.center(), (rightBounds.center() - rightBounds.min()).length() );
            rightChild.m_volumeIndex = m_volumes.size();
            m_volumes.push_back(rightSphere);

            node.m_leftChildIndex = m_nodes.size();
            leftChildIndex = node.m_leftChildIndex;
            rightChildIndex = leftChildIndex + 1;
            m_nodes.push_back(leftChild);
            m_nodes.push_back(rightChild);
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
	s.m_center = bounds.center();
	s.m_radius = (bounds.center() - bounds.min()).length();
	return s;
}

bool BVH::intersection(const RenderLib::Raytracing::Ray &r,
						const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices,
						RenderLib::Math::Vector3f& isect) const {
    return intersection_r(r, vertices, indices, 0, isect);
}

bool BVH::intersection_r(const RenderLib::Raytracing::Ray& r,
                         const std::vector<RenderLib::Math::Vector3f>& vertices, const std::vector<int>& indices,
                         int node,
                         RenderLib::Math::Vector3f& isect) const {
	using namespace RenderLib::Math;
	using namespace RenderLib::Raytracing;
	using namespace RenderLib::Geometry;

    const BVHNode& n = m_nodes[node];
    float t;
    if( !m_volumes[n.m_volumeIndex].intersection(r, t)) return false;

    // got intersection
    if( n.m_leaf ) {
        // we have an intersection with a leaf volume, refine with the actual primitive.
        const Vector3f& a = vertices[ indices[ 3 * n.m_primitiveIndex ] ];
        const Vector3f& b = vertices[ indices[ 3 * n.m_primitiveIndex + 1 ] ];
        const Vector3f& c = vertices[ indices[ 3 * n.m_primitiveIndex + 2 ] ];
        const Sphere& volume = m_volumes[ n.m_volumeIndex ];
        const Point3f q = r.origin + r.direction * ((r.origin - volume.m_center).length() + volume.m_radius);
        return segmentTriIntersection( r.origin, q, a, b, c, isect);
    } else {
        // sort children by distance to ray origin
        // and test in that order (hoping the closest one
        // will intersect first)
        const int leftChildIdx = n.m_leftChildIndex;
        const int rightChildIdx = leftChildIdx + 1;
        const Sphere& leftChildVolume = m_volumes[ m_nodes[leftChildIdx].m_volumeIndex ];
        const Sphere& rightChildVolume = m_volumes[ m_nodes[rightChildIdx].m_volumeIndex ];
        float distLeft = Vector3f::dot(r.direction, leftChildVolume.m_center - r.origin);
        float distRight = Vector3f::dot(r.direction, rightChildVolume.m_center - r.origin);
        if ( distLeft <= distRight ) {
            // recurse first left, then right
            if (intersection_r(r, vertices, indices, leftChildIdx, isect)) return true;
            return intersection_r(r, vertices, indices, rightChildIdx, isect);
        } else {
            // recurse first right, then left
            if (intersection_r(r, vertices, indices, rightChildIdx, isect)) return true;
            return intersection_r(r, vertices, indices, leftChildIdx, isect);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////

inline float scalarTriple( const RenderLib::Math::Point3f& u, const RenderLib::Math::Point3f& v, const RenderLib::Math::Point3f& w ) {
	return RenderLib::Math::Vector3f::dot(RenderLib::Math::Vector3f::cross(u.fromOrigin(),v.fromOrigin()), w.fromOrigin());
}

// This expects triangles in CCW order
bool segmentTriIntersection( const RenderLib::Math::Point3f& p, const RenderLib::Math::Point3f& q,
                             const RenderLib::Math::Point3f& a, const RenderLib::Math::Point3f& b, const RenderLib::Math::Point3f& c,
                             RenderLib::Math::Vector3f& isect) {
	using namespace RenderLib::Math;
	using namespace RenderLib::Raytracing;
	using namespace RenderLib::Geometry;

    const Point3f pq = q - p;
    const Point3f pa = a - p;
    const Point3f pb = b - p;
    const Point3f pc = c - p;

    // test if pq is inside the edges bc, ca and ab. Done by testing
    // that the signed tetrahedral volumes, computed using scalar triple
    // products, are all positive
    float u = scalarTriple(pq, pc, pb);
    if (u < 0.0f) return false;
    float v = scalarTriple(pq, pa, pc);
    if (v < 0.0f) return false;
    float w = scalarTriple(pq, pb, pa);
    if (w < 0.0f) return false;

    u = std::max(u, 0.0f);
    v = std::max(v, 0.0f);
    w = std::max(w, 0.0f);

    // compute the varycentric coordinates determining the
    // intersection point, r
    float denom = 1.0f / (u+v+w);
    u *= denom;
    v *= denom;
    w *= denom;
    assert( fabsf(w - 1.0f + u + v ) < 1e-3f );
    assert( u >= 0.0f && u <= 1.0f );
    assert( v >= 0.0f && v <= 1.0f );
    assert( w >= 0.0f && w <= 1.0f );
    isect = a.fromOrigin() * u + b.fromOrigin() * v + c.fromOrigin() * w;
    return true;
}

} // namespace DataStructures
} // namespace RenderLib