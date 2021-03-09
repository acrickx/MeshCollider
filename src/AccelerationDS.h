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
        float dmin = 0;
        vec3 C = other.c;

        for (int i = 0; i < 3; i++) {
            if (C[i] < m_minCorner[i]) dmin += sqrt(C[i] - m_minCorner[i]); 
            else if (C[i] > m_maxCorner[i]) dmin += sqrt(C[i] - m_maxCorner[i]);
        }
        if (dmin <= powf(other.r, 2))
        {            
            return true;
        }
        else return false;
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
private:
    //std::shared_ptr<BVHnode> m_left=nullptr;
    //std::shared_ptr<BVHnode> m_right=nullptr;
    BVHnode *m_left = nullptr;
    BVHnode *m_right = nullptr;
    AABB m_aabb;
    buffer<uint3> m_connectivity;
    const mesh* m_mesh=nullptr;

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
        //std::cout << "size : " << connectivity.size() << std::endl;
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
            vec3 init(mod->position(connectivity(0)[0]));
            vec3 minRight(init), minLeft(init), maxRight(init), maxLeft(init);
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
            m_left = new BVHnode(connectivityLeft, aabbLeft, mod, sizeScale);
            m_right = new BVHnode(connectivityRight, aabbLeft, mod, sizeScale);       
        }
    }

    inline void computeSurroundingSphere(const AABB& aabb, vec3& centerSphere, float& radius) {
        radius = norm(aabb.maxCorner() - aabb.minCorner()) / 2.f;
        centerSphere = (aabb.minCorner() + aabb.maxCorner()) / 2.f;
    }

    // intersection between 2 objects
    inline bool intersect (const BVHnode& other, buffer<SS>& triangleSS, buffer<SS>& triangleSSOther)
    {
        if (!m_aabb.intersect(other.aabb()))
            return false;
        if (isLeaf() && other.isLeaf()) {
            vec3 centerSphere, centerOther;
            float radius, radiusOther;
            computeSurroundingSphere(m_aabb, centerSphere, radius);
            computeSurroundingSphere(other.m_aabb, centerOther, radiusOther);
            if (norm(centerSphere - centerOther) <= radius + radiusOther) {
                SS ssThis, ssOther;
                ssThis.center = centerSphere; ssOther.center = centerOther;
                ssThis.radius = radius; ssOther.radius = radiusOther;
                triangleSS.push_back(ssThis);
                triangleSSOther.push_back(ssOther);
                return true;
            }
            return false;
        }
        if (!isLeaf())
        {
            if (!other.isLeaf())
            {
                bool ll = m_left->intersect(*(other.left()), triangleSS, triangleSSOther);
                bool lr = m_left->intersect(*(other.right()), triangleSS, triangleSSOther);
                bool rr = m_right->intersect(*(other.right()), triangleSS, triangleSSOther);
                bool rl = m_right->intersect(*(other.left()), triangleSS, triangleSSOther);
                return ll || lr || rr || rl;
            }
            else
            {
                bool lo = m_left->intersect(other, triangleSS, triangleSSOther);
                bool ro = m_right->intersect(other, triangleSS, triangleSSOther);
                return lo || ro;
            }
        }
        else
        {
            bool tl = intersect(*(other.left()), triangleSS, triangleSSOther);
            bool tr = intersect(*(other.right()), triangleSS, triangleSSOther);
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

    //intersect with sphere
    inline bool intersect(particle_structure& part) {        
        if (!m_aabb.intersect(part))
            return false;
        if (isLeaf()) {
            std::cout << "intersect" << std::endl;
            SS aabb_sphere;
            computeSurroundingSphere(m_aabb, aabb_sphere.center, aabb_sphere.radius);
            //contact normal
            vec3 n(normalize(part.c - aabb_sphere.center));
            //position
            part.p = aabb_sphere.center + (aabb_sphere.radius+ part.r)*n;
            //velocity
            vec3 const vn = dot(part.v, n) * n;
            vec3 const vt = part.v - vn;
            part.v = -0.95f * vn + 0.9 * vt;
            return true;
        }
        bool intersectLeft = m_left->intersect(part);
        bool intersectRight = m_right->intersect(part);
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
    inline const BVHnode* left() const { return m_left; }
    inline const BVHnode* right() const { return m_right; }
    inline BVHnode* left() { return m_left; }
    inline BVHnode* right() { return m_right; }
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
            m_mesh.position(i) = m_mesh.position(i)+translation;
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
    float m_sizeScale = 0.1f;
};

void simulate(std::vector<model*>& particles, float dt);