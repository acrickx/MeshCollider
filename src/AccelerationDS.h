#pragma once

#include "vcl/vcl.hpp"
#include "simulation.hpp"

#include<algorithm>

using namespace vcl;

class AABB {
public:
    inline AABB() {};

    inline AABB(const mesh& initMesh) {
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
    }

    inline AABB(const vec3& mini, const vec3& maxi)
    {
        m_minCorner = mini;
        m_maxCorner = maxi;
    }

    inline bool intersect(const AABB& aabb2) {
        const vec3& min2 = aabb2.minCorner();
        const vec3& max2 = aabb2.maxCorner();
        if (m_maxCorner(0) < min2(0) || m_maxCorner(1) < min2(1) || m_maxCorner(2) < min2(2) ||
            m_minCorner(0) > max2(0) || m_minCorner(1) > max2(1) || m_minCorner(2) > max2(2))
            return false;
        return true;
    }

    inline vec3& minCorner() { return m_minCorner; }
    inline const vec3& minCorner() const { return m_minCorner; }

    inline vec3& maxCorner() { return m_maxCorner; }
    inline const vec3& maxCorner() const { return m_maxCorner; }

private:
    vec3 m_minCorner;
    vec3 m_maxCorner;
};

class BVHnode {
private:
    BVHnode* m_left=nullptr;
    BVHnode* m_right=nullptr;
    AABB m_aabb;
    buffer<uint3> m_connectivity;
    const mesh* m_mesh=nullptr;

public:    
    inline BVHnode() {}
    inline BVHnode(const mesh* mod)
    {
        AABB aabb(*mod);
        buffer<uint3> connectivity = mod->connectivity;
        new (this) BVHnode(connectivity, aabb, mod);
    }
    inline BVHnode(buffer<uint3> connectivity, AABB aabb, const mesh* mod)
    {
        std::cout << "size : " << connectivity.size() << std::endl;
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
            const vec3& init = mod->position(connectivity(0)[0]);
            vec3 minRight=init, minLeft=init, maxRight=init, maxLeft=init;
            for (int i = 0; i < connectivity.size(); i++)
            {            
                const vec3& pt0 = mod->position(connectivity(i)[0]);
                const vec3& pt1 = mod->position(connectivity(i)[1]);
                const vec3& pt2 = mod->position(connectivity(i)[2]);
                vec3 bar = (pt0 + pt1 + pt2) / 3.f;
                if (bar(dimension) < median)
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
            AABB aabbLeft(minLeft, maxLeft), aabbRight(minRight, maxRight);
            BVHnode left(connectivityLeft, aabbLeft, mod), right(connectivityRight, aabbRight, mod);
            m_left = &left; m_right = &right;        
        }
    }
};


class model {
public:
    inline model() {}

    inline model(mesh initMesh) {        
        m_mesh = initMesh;
        m_BVHroot = BVHnode(&m_mesh);
    }

    inline mesh& modelMesh() { return m_mesh; }
    inline const mesh& modelMesh() const { return m_mesh; }

    inline BVHnode& BVHroot() { return m_BVHroot; }

    inline const BVHnode& BVHroot() const { return m_BVHroot; }

private:
    BVHnode m_BVHroot;
    mesh m_mesh;
};
