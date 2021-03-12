#pragma once

#include "vcl/vcl.hpp"
#include<algorithm>

using namespace vcl;

struct particle_structure
{
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};

class AABB {
public:
    inline AABB() {};

    inline AABB(const mesh& initMesh, const float sizeScale) {
        const buffer<vec3>& vertices = initMesh.position;
        m_minCorner = vertices(0);
        m_maxCorner = vertices(0);
        for (size_t i = 0; i < vertices.size(); i++) {
            const vec3& vertex = vertices(i);
            m_minCorner(0) = std::min(vertex(0), m_minCorner(0));
            m_minCorner(1) = std::min(vertex(1), m_minCorner(1));
            m_minCorner(2) = std::min(vertex(2), m_minCorner(2));
            m_maxCorner(0) = std::max(vertex(0), m_maxCorner(0));
            m_maxCorner(1) = std::max(vertex(1), m_maxCorner(1));
            m_maxCorner(2) = std::max(vertex(2), m_maxCorner(2));
        }
        m_minCorner *= sizeScale;
        m_maxCorner *= sizeScale;
    }

    inline AABB(const vec3& mini, const vec3& maxi, const float sizeScale)
    {
        m_minCorner = vec3(mini) * sizeScale;
        m_maxCorner = vec3(maxi) * sizeScale;
    }

    inline bool intersect(const AABB& aabb2) {
        const vec3& min2 = aabb2.minCorner();
        const vec3& max2 = aabb2.maxCorner();
        if (m_maxCorner(0) < min2(0) || m_maxCorner(1) < min2(1) || m_maxCorner(2) < min2(2) ||
            m_minCorner(0) > max2(0) || m_minCorner(1) > max2(1) || m_minCorner(2) > max2(2))
            return false;
        return true;
    }

    inline bool intersect(const particle_structure& other)
    {
        vec3 S = other.p; vec3 C1 = m_minCorner; vec3 C2 = m_maxCorner;
        float dist_squared = other.r * other.r;
        float dmin = 0.f;
        for (int i = 0; i <= 2; i++) {
            if (S(i) < C1(i)) dmin += (S(i) - C1(i)) * (S(i) - C1(i));
            else if (S(i) > C2(i)) dmin += (S(i) - C2(i)) * (S(i) - C2(i));
        }
        return dist_squared > dmin - 1e-5;
    }

    // intersection with plane
    inline bool intersect(vec3 const& n, vec3 const& p0) {
        vec3 mid = (m_minCorner + m_maxCorner) / 2.f;
        vec3 posExtends = m_maxCorner - mid;
        float r = posExtends(0) * std::abs(n(0)) + posExtends(1) * std::abs(n(1)) + posExtends(2) * std::abs(n(2));
        float s = std::abs(dot(n, mid)) - norm(p0);
        s = std::abs(s);
        return s <= r;
    }

    inline void updatePosition(const vec3& position) {
        m_minCorner += vec3(position);
        m_maxCorner += vec3(position);
    }

    inline vec3& minCorner() { return m_minCorner; }
    inline const vec3& minCorner() const { return m_minCorner; }

    inline vec3& maxCorner() { return m_maxCorner; }
    inline const vec3& maxCorner() const { return m_maxCorner; }

private:
    vec3 m_minCorner;
    vec3 m_maxCorner;
};

struct SS {
    vec3 center;
    float radius = 0.f;
};

class BVHnode {
    using BVHptr = std::shared_ptr<BVHnode>;
    typedef std::shared_ptr<BVHnode> BVHptr;
private:
    //std::shared_ptr<BVHnode> m_left=nullptr;
    //std::shared_ptr<BVHnode> m_right=nullptr;
    BVHptr m_left = nullptr;
    BVHptr m_right = nullptr;
    AABB m_aabb;
    buffer<uint3> m_connectivity;
    const mesh* m_mesh = nullptr;

public:
    inline BVHnode() {}
    inline BVHnode(const mesh* mod, const float sizeScale)
    {
        AABB aabb(*mod, sizeScale);
        buffer<uint3> rootConnectivity = mod->connectivity;
        new (this) BVHnode(rootConnectivity, aabb, mod, sizeScale);
    }
    inline BVHnode(buffer<uint3> connectivity, AABB aabb, const mesh* mod, const float sizeScale)
    {
        //stop condition
        if (connectivity.size() <= 1)
        {
            m_connectivity = connectivity;
            m_aabb = aabb;
            m_mesh = mod;            
        }
        else
        {
            m_connectivity = connectivity;
            m_aabb = aabb;
            m_mesh = mod;
            //determine in which dimension to split 
            vec3 diff = aabb.maxCorner() - aabb.minCorner();
            int dimension=-1;
            if (diff(0) >= std::max(diff(1), diff(2))) dimension = 0;
            else if (diff(1) >= std::max(diff(0), diff(2))) dimension = 1;
            else if (diff(2) >= std::max(diff(1), diff(0))) dimension = 2;   
            std::vector<float> barycenters;
            //compute triangle barycenter
            for (int i = 0; i < connectivity.size(); i++)
            {
                const vec3& pt0 = mod->position(connectivity(i)[0]);
                const vec3& pt1 = mod->position(connectivity(i)[1]);
                const vec3& pt2 = mod->position(connectivity(i)[2]);
                vec3 bar = (pt0 + pt1 + pt2) / 3.f;
                barycenters.push_back(bar(dimension));
            }
            std::sort(barycenters.begin(), barycenters.end());
            float median = barycenters[barycenters.size() / 2];
            //sort triangles in two subsets
            buffer<uint3> connectivityLeft, connectivityRight;
            constexpr float MAX_FLOAT = std::numeric_limits<float>::max();
            constexpr float MIN_FLOAT = std::numeric_limits<float>::lowest();
            vec3 minRight(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT), minLeft(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
            vec3 maxRight(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT), maxLeft(MIN_FLOAT, MIN_FLOAT, MIN_FLOAT);
            bool addLeft = false;
            for (int i = 0; i < connectivity.size(); i++)
            {            
                const vec3& pt0 = mod->position(connectivity(i)[0]);
                const vec3& pt1 = mod->position(connectivity(i)[1]);
                const vec3& pt2 = mod->position(connectivity(i)[2]);
                vec3 bar = (pt0 + pt1 + pt2) / 3.f;
                if (bar(dimension) == median)
                {
                    if (addLeft) {
                        connectivityLeft.push_back(connectivity(i));
                        for (int j = 0; j < 3; j++)
                        {
                            minLeft(j) = std::min(minLeft(j), std::min(pt2(j), std::min(pt0(j), pt1(j))));
                            maxLeft(j) = std::max(maxLeft(j), std::max(pt2(j), std::max(pt0(j), pt1(j))));
                        }
                    }
                    else {
                        connectivityRight.push_back(connectivity(i));
                        for (int j = 0; j < 3; j++)
                        {
                            minRight(j) = std::min(minRight(j), std::min(pt2(j), std::min(pt0(j), pt1(j))));
                            maxRight(j) = std::max(maxRight(j), std::max(pt2(j), std::max(pt0(j), pt1(j))));
                        }
                    }
                    addLeft = !addLeft;
                }
                else if (bar(dimension) < median)
                {
                    connectivityLeft.push_back(connectivity(i));
                    for (int j = 0; j < 3; j++)
                    {
                        minLeft(j) = std::min(minLeft(j),std::min(pt2(j),std::min(pt0(j), pt1(j))));   
                        maxLeft(j) = std::max(maxLeft(j),std::max(pt2(j),std::max(pt0(j), pt1(j))));   
                    }
                }
                else
                {
                    connectivityRight.push_back(connectivity(i));
                    for (int j = 0; j < 3; j++)
                    {
                        minRight(j) = std::min(minRight(j), std::min(pt2(j), std::min(pt0(j), pt1(j))));
                        maxRight(j) = std::max(maxRight(j), std::max(pt2(j), std::max(pt0(j), pt1(j))));
                    }
                }
            }
            //compute AABB
            AABB aabbLeft(minLeft, maxLeft, sizeScale), aabbRight(minRight, maxRight, sizeScale);
            m_left = BVHptr(new BVHnode(connectivityLeft, aabbLeft, mod, sizeScale));
            m_right = BVHptr(new BVHnode(connectivityRight, aabbRight, mod, sizeScale));       
        }
    }

    inline void computeSurroundingSphere(const AABB& aabb, vec3& centerSphere, float& radius) {
        radius = norm(aabb.maxCorner() - aabb.minCorner()) / 2.f;
        centerSphere = (aabb.minCorner() + aabb.maxCorner()) / 2.f;
    }

    // intersection between 2 objects
    inline bool intersect (const BVHnode& other, buffer<AABB>& triangleAABB, buffer<AABB>& otherAABB,
        buffer<uint3>& tri, buffer<uint3>& triOther)
    {
        if (!m_aabb.intersect(other.aabb()))
            return false;
        if (isLeaf() && other.isLeaf()) {
            tri.push_back(m_connectivity(0));
            triOther.push_back(other.connectivity()(0));
            triangleAABB.push_back(m_aabb);
            otherAABB.push_back(other.aabb());
            return true;
        }
        if (!isLeaf())
        {
            if (!other.isLeaf())
            {
                bool ll = m_left->intersect(*(other.left()), triangleAABB, otherAABB, tri, triOther);
                bool lr = m_left->intersect(*(other.right()), triangleAABB, otherAABB, tri, triOther);
                bool rr = m_right->intersect(*(other.right()), triangleAABB, otherAABB, tri, triOther);
                bool rl = m_right->intersect(*(other.left()), triangleAABB, otherAABB, tri, triOther);
                return ll || lr || rr || rl;
            }
            else
            {
                bool lo = m_left->intersect(other, triangleAABB, otherAABB, tri, triOther);
                bool ro = m_right->intersect(other, triangleAABB, otherAABB, tri, triOther);
                return lo || ro;
            }
        }
        else
        {
            bool tl = intersect(*(other.left()), triangleAABB, otherAABB, tri, triOther);
            bool tr = intersect(*(other.right()), triangleAABB, otherAABB, tri, triOther);
            return tl || tr;
        }
    }

    // intersection with plane (aabb intersection test)
    inline bool intersect(vec3 const& n, vec3 const& p0, buffer<AABB*>& aabbs) {
        if (!m_aabb.intersect(n, p0))
            return false;
        if (isLeaf()) {
            aabbs.push_back(&m_aabb);
            return true;
        }
        bool intersectLeft = m_left->intersect(n, p0, aabbs);
        bool intersectRight = m_right->intersect(n, p0, aabbs);
        return intersectLeft || intersectRight;
    }

    //intersect with sphere (assumes the object is static)
    inline bool intersect(particle_structure& part, buffer<vec3> &newPos, buffer<vec3>& newVt, buffer<vec3>& newVn) {
        if (!m_aabb.intersect(part))
            return false;
        if (isLeaf()) {
            vec3 S = part.p; vec3 C1 = m_aabb.minCorner(); vec3 C2 = m_aabb.maxCorner();
            float dmin = 0.f;
            for (int i = 0; i <= 2; i++) {
                if (S(i) < C1(i)) dmin += (S(i) - C1(i)) * (S(i) - C1(i));
                else if (S(i) > C2(i)) dmin += (S(i) - C2(i)) * (S(i) - C2(i));
            }
            float dpen = part.r - sqrt(dmin);
            const vec3& p1 = m_mesh->position[m_connectivity(0)(0)];
            const vec3& p2 = m_mesh->position[m_connectivity(0)(1)];
            const vec3& p3 = m_mesh->position[m_connectivity(0)(2)];
            vec3 nTri = cross(p2 - p1, p3 - p1);
            nTri /= norm(nTri);
            vec3 baryTri = (p1 + p2 + p3) / 3.f;
            if (dot(nTri, baryTri - part.p) > 0)
                nTri = -nTri;
            int dim;
            if (std::abs(nTri(0)) >= std::max(std::abs(nTri(1)), std::abs(nTri(2))))
                dim = 0;
            else if (std::abs(nTri(1)) >= std::max(std::abs(nTri(0)), std::abs(nTri(2))))
                dim = 1;
            else
                dim = 2;
            //position
            vec3 trans;
            trans(dim) = dpen;
            if (nTri(dim) < 0)
                trans(dim) = -dpen;
            newPos.push_back(part.p + trans);
            //velocity
            vec3 const vn = dot(part.v, nTri) * nTri;
            vec3 const vt = part.v - vn;
            newVt.push_back(vt);
            newVn.push_back(vn);
            return true;
        }
        bool intersectLeft = m_left->intersect(part, newPos, newVt, newVn);
        bool intersectRight = m_right->intersect(part, newPos, newVt, newVn);
        return intersectLeft || intersectRight;
    }

    // intersection with plane (triangle intersection test)
    inline bool intersect(vec3 const& n, vec3 const& p0, buffer<uint3*>& triangles, const vec3& trans, float sizeScale) {
        if (!m_aabb.intersect(n, p0))
            return false;
        if (isLeaf()) {
            const vec3& p1 = m_mesh->position(m_connectivity(0)(0)) * sizeScale + trans;
            const vec3& p2 = m_mesh->position(m_connectivity(0)(1)) * sizeScale + trans;
            const vec3& p3 = m_mesh->position(m_connectivity(0)(2)) * sizeScale + trans;

            float dot1 = dot(p0 - p1, n), dot2 = dot(p0 - p2, n), dot3 = dot(p0 - p3, n);
            if ((dot1 < 0 && dot2 < 0 && dot3 < 0) || (dot1 > 0 && dot2 > 0 && dot3 > 0))
                return false;
            triangles.push_back(&m_connectivity(0));
            return true;
        }
        bool intersectLeft = m_left->intersect(n, p0, triangles, trans, sizeScale);
        bool intersectRight = m_right->intersect(n, p0, triangles, trans, sizeScale);
        return intersectLeft || intersectRight;
    }

    inline void updatePosition(const vec3& position) {
        m_aabb.updatePosition(position);
        if (!isLeaf()) {
            m_left->updatePosition(position);
            m_right->updatePosition(position);
        }
    }

    inline bool isLeaf() const { return (m_connectivity.size() == 1); }    
    inline const AABB& aabb() const { return m_aabb; }
    inline AABB& aabb() { return m_aabb; }
    inline const BVHptr left() const { return m_left; }
    inline const BVHptr right() const { return m_right; }
    inline BVHptr left() { return m_left; }
    inline BVHptr right() { return m_right; }
    inline buffer<uint3>& connectivity() { return m_connectivity; }
    inline const buffer<uint3>& connectivity() const { return m_connectivity; }
};


class model {
public:
    inline model() {}

    inline model(mesh initMesh) {        
        m_mesh = initMesh;
        m_BVHroot = BVHnode(&m_mesh, m_sizeScale);
    }

    inline model(const model& other) {
        m_mesh = other.m_mesh;
        m_BVHroot = BVHnode(&m_mesh, other.m_sizeScale);
        m_position = other.m_position;
        m_velocity = other.m_velocity;
        m_color = other.m_color;
        m_mass = other.m_mass;
        m_sizeScale = other.m_sizeScale;
    }

    inline void updatePosition(const vec3& newPos) {
        m_BVHroot.updatePosition(newPos - m_position);
        m_position = vec3(newPos);
    }

    inline void rotate(const rotation& rot) {
        for (size_t i = 0; i < m_mesh.position.size(); i++) {
            m_mesh.position(i) = rot * m_mesh.position(i);
        }
    }

    inline void scale(const float scaleFactor) {
        for (size_t i = 0; i < m_mesh.position.size(); i++) {
            m_mesh.position(i) = scaleFactor * m_mesh.position(i);
        }
    }

    inline void translate(const vec3& translation) {
        for (size_t i = 0; i < m_mesh.position.size(); i++) {
            m_mesh.position(i) = m_mesh.position(i) + translation;
        }
    }

    inline mesh& modelMesh() { return m_mesh; }
    inline const mesh& modelMesh() const { return m_mesh; }

    inline BVHnode& BVHroot() { return m_BVHroot; }
    inline const BVHnode& BVHroot() const { return m_BVHroot; }

    inline vec3& position() { return m_position; }
    inline const vec3& position() const { return m_position; }

    inline vec3& velocity() { return m_velocity; }
    inline const vec3& velocity() const { return m_velocity; }

    inline vec3& color() { return m_color; }
    inline const vec3& color() const { return m_color; }

    inline float& mass() { return m_mass; }
    inline const float& mass() const { return m_mass; }

    inline float& sizeScale() { return m_sizeScale; }
    inline const float& sizeScale() const { return m_sizeScale; }

private:
    BVHnode m_BVHroot;
    mesh m_mesh;
    vec3 m_position; // position
    vec3 m_velocity; // velocity
    vec3 m_color; // color
    float m_mass = 1.f; // mass
    float m_sizeScale = 1.f;
};

void simulate(std::vector<model*>& particles, float dt);