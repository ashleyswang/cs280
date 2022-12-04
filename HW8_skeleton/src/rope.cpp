#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    const float DAMPING_FACTOR = 0.00005;

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D curr = start;
        auto increment = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; ++i) {
            Mass* m = new Mass(curr, node_mass, false);
            masses.push_back(m);
            curr += increment;

            if (i == 0) continue;
            Spring* s = new Spring(masses[i - 1], masses[i], k);
            springs.push_back(s);
        }

        for (const int i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ab = s->m2->position - s->m1->position;
            Vector2D f = s->k * ab.unit() * (ab.norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces -= f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity;
                Vector2D a = m->forces / m->mass;
                
                /* Explicit Euler */
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;

                /* Implicit Euler */
                m->velocity += a * delta_t;                
                m->position += m->velocity * delta_t;
            }
            // TODO (Part 2): Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet
            Vector2D ab = s->m2->position - s->m1->position;

            // TODO (Part 3): Move each mass by the half displacement
            auto disp = 0.5 * ab.unit() * (ab.norm() - s->rest_length);
            if(s->m1->pinned && s->m2->pinned) continue;
            s->m1->position += s->m1->pinned ? Vector2D(0,0) : disp;
            s->m2->position -= s->m2->pinned ? Vector2D(0,0) : disp;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += m->mass * gravity;
                Vector2D a = m->forces / m->mass;
                Vector2D temp_position = m->position;

                // TODO (Part 4): Add global Verlet damping
                m->position += (1 - DAMPING_FACTOR) * (m->position - m->last_position) + a * delta_t * delta_t; 
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
