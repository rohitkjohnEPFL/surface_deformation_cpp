#include "interaction.hpp"
#include <vector>
#include <string>
#include <map>
#include <utility>
#include <stdexcept>
#include <iterator>

class BodyContainer
{
    private:
        std::vector<Body> bodies;
        int next_id;

    public:
        BodyContainer() : bodies(), next_id(0) {}

        void add_Body(
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
                );

        Body& operator[](int body_id);

        std::vector<Body>::iterator begin() { return bodies.begin(); }

        std::vector<Body>::iterator end() { return bodies.end(); }
};



class InteractionContainer {
private:
    std::map<std::pair<int, int>, Interaction> interactions;

public:
        void add_Interaction(
                Body b1, 
                Body b2, 
                double dt, 
                double young, 
                double poisson, 
                double damping_coeff
                );

    Interaction& operator[](const std::pair<int, int>& ids);

    auto begin() -> decltype(interactions.begin()) { return interactions.begin(); }

    auto end() -> decltype(interactions.end()) { return interactions.end(); }
};



class Scene
{
    private:
            BodyContainer bodies;
            InteractionContainer interactions;
            int iteration;

            // Private constructor so that no objects can be created.
            Scene() : bodies(), interactions(), iteration(1) {}
            static Scene* instance;

    public:
        // Delete the copy constructor and assignment operators
        Scene(Scene &other) = delete;
        void operator=(const Scene &) = delete;

        // Static access method
        static Scene* getInstance();

        BodyContainer& get_Bodies() { return bodies; }

        InteractionContainer& get_Interactions() { return interactions; }

        int get_Iteration() { return iteration; }

        void increment_Iteration() { iteration++; }
};



// Abstract base class
class SerialEngine {
private:
    int iterPeriod;
    Scene* scene_ptr;  // Pointer to the singleton Scene instance

public:
    // Constructor
    SerialEngine() : iterPeriod(1), scene_ptr(Scene::getInstance()) {}
    SerialEngine(int iterPeriod) : iterPeriod(iterPeriod), scene_ptr(Scene::getInstance()) {}

    // Getters
    int getIterPeriod() const { return iterPeriod; }
    Scene* getScenePtr() const { return scene_ptr; }

    // Abstract method, to be overridden by derived classes
    virtual void run() = 0;
};



class ForceResetter : public SerialEngine 
{
    public:
        // Constructor
        ForceResetter() : SerialEngine() {}
        ForceResetter(int iterPeriod) : SerialEngine(iterPeriod) {}

        // Override run method
        void run() override {
            for (Body& body : getScenePtr()->get_Bodies()) {
                body.reset_ForceTorque();
            }
        }
};


class InteractionsCalculator : public SerialEngine
{
    private:
        InteractionContainer interactionList;

    public:
        // Constructor
        InteractionsCalculator() : SerialEngine() {}
        InteractionsCalculator(int iterPeriod) ;


        // Override run method
        void run() override {
            for (auto& interaction : interactionList) {
                interaction.second.calc_ForcesTorques();
            }
        }

        void initialRun() {
            for (auto& interaction : interactionList) {
                interaction.second.calc_initForcesTorques();
            }
        }
};


class LeapFrogIntegrator : public SerialEngine
{
    private:
        double dt;
        Vector3D gravity;

    public:
        // Constructor
        LeapFrogIntegrator() : SerialEngine(), dt(1e-6), gravity({0, 0, -9.81}) {}
        LeapFrogIntegrator(int iterPeriod, double dt, Vector3D gravity) : SerialEngine(iterPeriod), dt(dt), gravity(gravity) {}

        // Override run method
        void run() override ;
};



class CustomFunctionEngine : public SerialEngine
{
    private:
        void (*function)(void);


    public:
        // Constructor
        CustomFunctionEngine() : SerialEngine(), function(nullptr) {}
        CustomFunctionEngine(void (*pyFunction)(void)) : SerialEngine(), function(pyFunction) {}

        // Override run method
        void run() override { function(); }
};


class SimulationLoop
{
    private:
        std::vector<SerialEngine*> engines;
        Scene* scene_ptr;

    public:
        // Constructor
        SimulationLoop() : engines(), scene_ptr(Scene::getInstance()) {}

        void simulate(int no_iterations) ;

        void add_Engine(SerialEngine* engine) { engines.push_back(engine); }
};

