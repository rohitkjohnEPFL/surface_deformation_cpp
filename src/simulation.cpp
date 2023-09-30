#include "simulation.hpp"

void BodyContainer::add_Body(
        double density, 
        double mass, 
        double radius, 
        double inertia, 
        bool DynamicQ, 
        Vector3D pos, 
        Vector3D vel, 
        Vector3D angVel, 
        Quaternion ori, 
        std::string BlockedDOFs
        )
{
    Body new_body(next_id, density, mass, radius, inertia, DynamicQ, pos, vel, angVel, ori, BlockedDOFs);
    bodies.push_back(new_body);
    next_id++;
}


Body& BodyContainer::operator[](int body_id)
{
    for (Body& body : bodies)
    {
        if (body.get_ID() == body_id)
        {
            return body;
        }
    }
    throw std::runtime_error("No body found with id " + std::to_string(body_id));
}


void InteractionContainer::add_Interaction(
        Body b1, 
        Body b2, 
        double dt, 
        double young, 
        double poisson, 
        double damping_coeff
        )
{
        // Ensure id1 is always smaller than id2
        int id1 = b1.get_ID(), id2 = b2.get_ID();

        if (id1 > id2) {
            std::swap(id1, id2);
        }

        Interaction new_interaction(b1, b2, dt, young, poisson, damping_coeff);
        interactions[std::make_pair(id1, id2)] = new_interaction;
}

Interaction& InteractionContainer::operator[](const std::pair<int, int>& ids) {
    int id1 = ids.first, id2 = ids.second;
    if (id1 > id2) {
        std::swap(id1, id2);
    }

    auto it = interactions.find(std::make_pair(id1, id2));
    if (it != interactions.end()) {
        return it->second;
    } else {
        throw std::runtime_error("Interaction not found");
    }
}


// scene
// Initialize pointer to nullptr (zero).
Scene* Scene::instance = nullptr;

Scene* Scene::getInstance() {
    if (instance == nullptr) {
        instance = new Scene();
    }
    return instance;
}


InteractionsCalculator::InteractionsCalculator(int iterPeriod) : SerialEngine(iterPeriod) 
{
    for (auto& interaction : interactionList) {
        interaction.second.reset_ForceTorque();
    }
}

void LeapFrogIntegrator::run()  {
    for (Body& body : getScenePtr()->get_Bodies()) {
        double mass  = body.mass;
        Vector3D pos0    = body.pos;
        Vector3D vel0    = body.vel;
        Vector3D angVel0 = body.angVel;
        Quaternion ori0  = body.ori;

        Vector3D acc0 = body.force / mass + gravity;
        Vector3D vel1 = vel0 + acc0 * dt * int(body.DynamicQ);
        Vector3D pos1 = pos0 + dt * vel1;

        Vector3D angVel1 = angVel0 + (dt * body.torque / body.inertia) * int(body.DynamicQ);
        double angVelMag = angVel1.get_Norm();
        double angle = dt * angVelMag;
        Quaternion delta_q;

        if (angle != 0.0) {
            Vector3D axis = angVel1 / angVelMag;
            double cos_t = std::cos(angle / 2);
            double sin_t = std::sin(angle / 2);
            delta_q = Quaternion({cos_t, axis.get_x() * sin_t, axis.get_y() * sin_t, axis.get_z() * sin_t});
        } else {
            delta_q = Quaternion({1, 0, 0, 0});
        }

        Quaternion ori1 = delta_q * ori0;

        body.pos   = pos1;
        body.vel   = vel1;
        body.ori   = ori1;
        body.angVel = angVel1;
    }
}

void SimulationLoop::simulate(int no_iterations) {
    for (int i = 1; i <= no_iterations; ++i) {
        for (SerialEngine* engine : engines) {
            if (i % engine->getIterPeriod() == 0) {
                engine->run();
            }
        }
        scene_ptr->increment_Iteration();
    }
}
