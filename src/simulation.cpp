#include "simulation.hpp"

using namespace vcl;

// new
void collision_obj_plane(model* obj, vcl::vec3 const& n, vcl::vec3 const& p0)
{
	float const epsilon = 1e-5f;
	float const alpha_n = 0.85f;  // attenuation normal
	float const alpha_t = 0.80f;  // attenuation tangential
	buffer<uint3*> triangles;
	if(obj->BVHroot().intersect(n, p0, triangles, obj->position(), obj->sizeScale())) {
		bool finished = false;
		vec3 newVel, translationPos;
		int tot = 0;
		while (!finished) {
			finished = true;
			for (size_t i = 0; i < triangles.size(); i++) {
				const uint3& tri = *triangles(i);
				const vec3 p1 = obj->modelMesh().position(tri(0)) * obj->sizeScale() + obj->position() + translationPos;
				const vec3 p2 = obj->modelMesh().position(tri(1)) * obj->sizeScale() + obj->position() + translationPos;
				const vec3 p3 = obj->modelMesh().position(tri(2)) * obj->sizeScale() + obj->position() + translationPos;

				float dot1 = dot(p0 - p1, n), dot2 = dot(p0 - p2, n), dot3 = dot(p0 - p3, n);
				bool update = true;
				float distPen = 0;
				if (dot1 > -epsilon) {
					if (dot2 < 0 && dot3 < 0)
						distPen = std::abs(dot1);
					else if (dot2 < 0 && dot3 > -epsilon)
						distPen = std::max(std::abs(dot1), std::abs(dot3));
					else if (dot3 < 0 && dot2 > -epsilon)
						distPen = std::max(std::abs(dot1), std::abs(dot2));
					else
						update = false;
				}
				else if (dot2 > -epsilon) {
					if (dot1 < 0 && dot3 < 0)
						distPen = std::abs(dot2);
					else if (dot1 < 0 && dot3 > -epsilon)
						distPen = std::max(std::abs(dot2), std::abs(dot3));
					else if (dot3 < 0 && dot1 > -epsilon)
						distPen = std::max(std::abs(dot2), std::abs(dot1));
					else
						update = false;
				}
				else if (dot3 > -epsilon) {
					if (dot1 < 0 && dot2 < 0)
						distPen = std::abs(dot3);
					else if (dot1 < 0 && dot2 > -epsilon)
						distPen = std::max(std::abs(dot3), std::abs(dot2));
					else if (dot2 < 0 && dot1 > -epsilon)
						distPen = std::max(std::abs(dot3), std::abs(dot1));
					else
						update = false;
				}
				if (update) {
					translationPos += distPen * n;
					vec3 const vn = dot(obj->velocity(), n) * n;
					vec3 const vt = obj->velocity() - vn;
					newVel += -alpha_n * vn + alpha_t * vt;
					tot++;
					//std::cout << p1 << " " << p2 << " " << p3 << "\n";
				}
			}
		}
		//obj->updatePosition(obj->position() + translationPos / float(tot));
		obj->updatePosition(obj->position() + translationPos);
		obj->velocity() = newVel / float(tot);
	}
}

void collision_sphere_plane(vcl::vec3& p, vcl::vec3& v, float r, vcl::vec3 const& n, vcl::vec3 const& p0)
{
    float const epsilon = 1e-5f;
    float const alpha_n = 0.95f;  // attenuation normal
    float const alpha_t = 0.90f;  // attenuation tangential

    float const s = dot(p-p0,n) - r;
    if( s<-epsilon )
    {
        vec3 const vn = dot(v,n) * n;
        vec3 const vt = v - vn;

        p = p - (s * n);
        v = -alpha_n * vn + alpha_t * vt;
    }
}



// new
void collision_obj_obj(model* obj1, model* obj2)
{
	float const epsilon = 1e-5f;
	float const alpha = 0.95f;
	buffer<SS> triangleSS;
	buffer<SS> triangleSSOther;
	if (obj1->BVHroot().intersect(obj2->BVHroot(), triangleSS, triangleSSOther)) {
		vec3 translationPos;
		vec3 newVel, newVelOther;
		bool finished = false;
		int tot = 0;
		while (!finished) {
			finished = true;
			for (size_t i = 0; i < triangleSS.size(); i++) {
				vec3 p1 = triangleSS(i).center + translationPos;
				vec3 p2 = triangleSSOther(i).center - translationPos;
				float r1 = triangleSS(i).radius;
				float r2 = triangleSSOther(i).radius;
				vec3 const p12 = p1 - p2;
				float const d12 = norm(p12);
				if (d12 < r1 + r2) {
					finished = false;
					vec3 const u12 = p12 / d12;
					float const collision_depth = r1 + r2 - d12;
					translationPos += (collision_depth / 2.0f + epsilon) * u12;
					tot++;
					if (norm(obj1->velocity() - obj1->velocity()) > 0.2f) {
						float const j = dot(newVel - newVelOther, u12);
						newVel -= alpha * j * u12;
						newVelOther += alpha * j * u12;
					}
					else // Contact
					{
						newVel /= 1.2f;
						newVelOther /= 1.2f;
					}
				}
			}
		}
		obj1->velocity() = newVel / float(tot);
		obj2->velocity() = newVelOther / float(tot);
		obj1->updatePosition(obj1->position() + translationPos);
		obj2->updatePosition(obj2->position() - translationPos);
	}
}

void collision_sphere_sphere(vcl::vec3& p1, vcl::vec3& v1, float r1, vcl::vec3& p2, vcl::vec3& v2, float r2)
{
	float const epsilon = 1e-5f;
    float const alpha = 0.95f;

    vec3 const p12 = p1-p2;
    float const d12 = norm(p12);

    if(d12 < r1+r2)
    {
        vec3 const u12 = p12/d12;
        float const collision_depth = r1+r2-d12;

        p1 += (collision_depth/2.0f+epsilon) * u12 ;
        p2 -= (collision_depth/2.0f+epsilon) * u12 ;

        if(norm(v1-v2)>0.2f){
            float const j = dot(v1-v2,u12);
            v1 = v1 - alpha*j*u12;
            v2 = v2 + alpha*j*u12;
        }
        else // Contact
        {
            v1 = v1/1.2f;
            v2 = v2/1.2f;
        }
    }
}

// new
void simulate(std::vector<model*>& objects, float dt_true) {
	vec3 const g = { 0,0,-9.81f };
	size_t const N_substep = 10;
	float const dt = dt_true / N_substep;

	for (size_t k_substep = 0; k_substep < N_substep; k_substep++)
	{
		size_t const N = objects.size();
		for (size_t k = 0; k < N; k++)
		{
			model* obj = objects[k];

			vec3 const f = obj->mass() * g;
			obj->velocity() = (1 - 0.9f * dt) * obj->velocity() + dt * f;
			const vec3 newPos = obj->position() + dt * obj->velocity();
			obj->updatePosition(newPos);
		}

		// Collisions between objects (without spacial acceleration structure)
		for (size_t k1 = 0; k1 < N; k1++)
		{
			for (size_t k2 = k1 + 1; k2 < N; k2++)
			{
				model* obj1 = objects[k1];
				model* obj2 = objects[k2];
				collision_obj_obj(obj1, obj2);
			}
		}
		// Collisions with cube
		const std::vector<vec3> face_normal = { {0, 1,0}, { 1,0,0}, {0,0, 1}, {0,-1,0}, {-1,0,0}, {0,0,-1} };
		const std::vector<vec3> face_position = { {0,-1,0}, {-1,0,0}, {0,0,-1}, {0, 1,0}, { 1,0,0}, {0,0, 1} };
		const size_t N_face = face_normal.size();
		for (size_t k = 0; k < N; ++k) {
			model* obj = objects[k];
			for (size_t k_face = 0; k_face < N_face; ++k_face)
				collision_obj_plane(obj, face_normal[k_face], face_position[k_face]);
		}
	}
}

void simulate(std::vector<particle_structure>& particles, float dt_true, std::vector<model>& objects)
{
	vec3 const g = {0,0,-9.81f};
	size_t const N_substep = 10;
	float const dt = dt_true/N_substep;
	for(size_t k_substep=0; k_substep<N_substep; ++k_substep)
	{
		size_t const N = particles.size();
		for (size_t k = 0; k < N; ++k)
		{
			particle_structure& particle = particles[k];
			vec3 const f = particle.m * g;
			particle.v = (1-0.9f*dt)*particle.v + dt*f;
			particle.p = particle.p + dt*particle.v;
		}

		// Collisions between spheres
		for(size_t k1=0; k1<N; ++k1)
		{
			for(size_t k2=k1+1; k2<N; ++k2)
			{
				particle_structure& p1 = particles[k1];
				particle_structure& p2 = particles[k2];

				collision_sphere_sphere(p1.p,p1.v,p1.r, p2.p,p2.v,p2.r);
			}
		}
		//Collisions with model
		for (size_t k = 0; k < N; k++)
		{
			for (int i = 0; i < objects.size(); i++)
			{				
				objects[i].BVHroot().intersect(particles[k]);
			}
		}

		// Collisions with cube
		const std::vector<vec3> face_normal  = {{0, 1,0}, { 1,0,0}, {0,0, 1}, {0,-1,0}, {-1,0,0}, {0,0,-1}};
		const std::vector<vec3> face_position = {{0,-1,0}, {-1,0,0}, {0,0,-1}, {0, 1,0}, { 1,0,0}, {0,0, 1}};
		const size_t N_face = face_normal.size();
		for(size_t k=0; k<N; ++k){
			particle_structure& part = particles[k];
			for(size_t k_face=0; k_face<N_face; ++k_face)
				collision_sphere_plane(part.p, part.v, part.r, face_normal[k_face], face_position[k_face]);
		}
	}
}