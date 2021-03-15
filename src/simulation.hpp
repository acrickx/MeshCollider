#pragma once

#include "vcl/vcl.hpp"
#include "AccelerationDS.h"

void simulate(std::vector<particle_structure>& particles, float dt, std::vector<model*>& objects);
void simulate(std::vector<model*>& models, float dt, bool reverse_g);
void collision_sphere_plane(vcl::vec3& p, vcl::vec3& v, float r, vcl::vec3 const& n, vcl::vec3 const& p0);
void collision_sphere_sphere(vcl::vec3& p1, vcl::vec3& v1, float r1, vcl::vec3& p2, vcl::vec3& v2, float r2);


