#pragma once

#include "vcl/vcl.hpp"
#include "simulation.hpp"

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


class BVH {

};


class model {
public:
    inline model() {}

    inline model(mesh initMesh) {
        m_aabb = AABB(initMesh);
        m_mesh = initMesh;
    }

    inline bool intersect(const model& model2) {
        return m_aabb.intersect(model2.aabb());
    }

    inline AABB& aabb() { return m_aabb; }

    inline const AABB& aabb() const { return m_aabb; }

private:
    AABB m_aabb;
    mesh m_mesh;
};
