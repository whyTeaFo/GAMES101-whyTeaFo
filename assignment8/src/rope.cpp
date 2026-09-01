#include <iostream>
#include <vector>
#include <algorithm>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

const double damping_fractor = 0.00005;

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (int i = 0; i < num_nodes; i++)
        {
            Vector2D pos = start + (end - start) * i / (num_nodes - 1);
            bool is_pinned = std::find(pinned_nodes.begin(), pinned_nodes.end(), i) != pinned_nodes.end();
            masses.push_back(new Mass(pos, node_mass, is_pinned));
        }

        for (int i = 0; i < num_nodes - 1; i++)
        {
            springs.push_back(new Spring(masses[i], masses[i + 1], k));
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D pa = s->m1->position;
            Vector2D pb = s->m2->position;
            double l = s->rest_length;
            Vector2D force = s->k * (pb - pa) / (pb-pa).norm() * ((pb-pa).norm() - l);
            s->m1->forces += force;
            s->m2->forces += -force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->velocity = m->velocity + (gravity+m->forces) * delta_t;
                m->last_position = m->position;
                m->position = m->last_position + m->velocity * delta_t;

                // (Part 2): Add global damping
                m->velocity *= 1.0 / (1.0 + damping_fractor * delta_t);
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D pa = s->m1->position;
            Vector2D pb = s->m2->position;
            double l = s->rest_length;
            s->m1->position = pa + (pb-pa) / (pb-pa).norm() * ((pb-pa).norm()-l) / 2;
            s->m2->position = pb + (pa-pb) / (pb-pa).norm() * ((pb-pa).norm()-l) / 2;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // (Part 3.1): Set the new position of the rope mass
                m->position = temp_position + (temp_position - m->last_position) + gravity * delta_t * delta_t;
                
                // TODO (Part 4): Add global Verlet damping
                m->position = temp_position + (1-damping_fractor) * (temp_position - m->last_position) + gravity * delta_t * delta_t;

                m->last_position = temp_position;
            }
        }
    }
}
