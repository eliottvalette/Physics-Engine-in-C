// gcc -o physics_engine_solver physics_engine_solver.c
// ./physics_engine_solver

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

// Two dimensional vector.
typedef struct {
    float x;
    float y;
} Vector2;

typedef struct {
    float x;
    float y;
    float z;
} Vector3;

typedef struct VerletObject VerletObject; // Forward declaration for function pointers

typedef void (*UpdateFunc)(VerletObject *obj, float dt);
typedef void (*AccelerateFunc)(VerletObject *obj, Vector2 a);
typedef Vector2 (*GetVelocityFunc)(VerletObject *obj, float dt);

// Verlet Object and its properties
struct VerletObject{
    Vector2 position;
    Vector2 position_last;
    Vector2 acceleration;
    float radius;
    int color;
    // Function pointers
    UpdateFunc update;
    AccelerateFunc accelerate;
    GetVelocityFunc getVelocity;
};

void objectUpdate(VerletObject *obj, float dt) {
    // Calculate displacement from last position
    Vector2 displacement = {
        obj->position.x - obj->position_last.x,
        obj->position.y - obj->position_last.y
    };

    // Store the current position before updating
    Vector2 current_position = obj->position;

    // Update position using Verlet integration formula
    obj->position.x += displacement.x + obj->acceleration.x * (dt * dt);
    obj->position.y += displacement.y + obj->acceleration.y * (dt * dt);

    // Update the last position to the previous position
    obj->position_last = current_position;

    // Reset acceleration for the next update
    obj->acceleration = (Vector2){0.0f, 0.0f};
}


void accelerate(VerletObject * obj, Vector2 a){
    obj->acceleration.x += a.x;
    obj->acceleration.y += a.y;
};

void setVelocity(VerletObject * obj, Vector2 v, float dt){
    obj->position_last.x = obj->position.x - (v.x * dt);
    obj->position_last.y = obj->position.y - (v.y * dt);
}

void addVelocity(VerletObject * obj, Vector2 v, float dt){
    obj->position_last.x -= v.x * dt;
    obj->position_last.y -= v.y * dt;
}

Vector2 getVelocity(VerletObject * obj, float dt){
    float x = (obj->position.x - obj->position_last.x) / dt;
    float y = (obj->position.y - obj->position_last.y) / dt;
    return (Vector2){x, y};
}


// Simulation functions
typedef struct {
    float time;
    float frame_dt;
    unsigned int sub_steps;
    Vector2 gravity;
    Vector2 constraint_center;
    float constraint_radius;
    VerletObject *objects;
    size_t object_count;
} Solver;

// Calculates the time step for each simulation sub-step
float getStepDt(Solver *solver) {
    return solver->frame_dt / (float)(solver->sub_steps);
}

// Sets the simulation update rate by modifying frame_dt in the solver
void setSimulationUpdateRate(Solver *solver, unsigned int rate) {
    solver->frame_dt = 1.0f / rate;
}

// Sets the constraint center and radius in the solver
void setConstraint(Solver *solver, Vector2 position, float radius) {
    solver->constraint_center = position;
    solver->constraint_radius = radius;
}

// Sets the velocity of a specific object
void setObjectVelocity(Solver *solver, VerletObject *obj, Vector2 v) {
    float dt = getStepDt(solver);
    setVelocity(obj, v, dt);
}

// Gets the constraint as a Vector3
Vector3 getConstraint(Solver *solver) {
    return (Vector3){solver->constraint_center.x, solver->constraint_center.y, solver->constraint_radius};
}

// Applies gravity to all objects in the solver
void applyGravity(Solver *solver) {
    for (size_t i = 0; i < solver->object_count; i++) {
        solver->objects[i].accelerate(&(solver->objects[i]), solver->gravity);
        printf("Applying gravity: Object %zu - Acceleration: (%f, %f)\n",
               i, solver->objects[i].acceleration.x, solver->objects[i].acceleration.y);
    }
}

// Updates all objects in the solver
void updateObjects(Solver *solver, float dt) {
    for (size_t i = 0; i < solver->object_count; i++) {
        solver->objects[i].update(&(solver->objects[i]), dt);
    }
}

void checkCollisions(Solver *solver, float dt) {
    const float response_coef = 0.75f; // Coefficient of restitution (how 'bouncy' collisions are)
    
    for (size_t i = 0; i < solver->object_count; i++) {
        VerletObject *obj1 = &solver->objects[i];
        for (size_t j = i + 1; j < solver->object_count; j++) {
            VerletObject *obj2 = &solver->objects[j];

            // Calculate the vector between the objects
            Vector2 v = {obj2->position.x - obj1->position.x, obj2->position.y - obj1->position.y};
            float dist2 = v.x * v.x + v.y * v.y;
            float min_dist = obj1->radius + obj2->radius;

            // Check if the objects are overlapping
            if (dist2 < min_dist * min_dist) {
                float dist = sqrtf(dist2);
                if (dist == 0.0f) dist = 0.01f; // Avoid division by zero

                // Normalize the vector
                Vector2 n = {v.x / dist, v.y / dist};

                // Mass ratios (assuming mass is proportional to area)
                float mass_ratio_1 = obj1->radius / (obj1->radius + obj2->radius);
                float mass_ratio_2 = obj2->radius / (obj1->radius + obj2->radius);

                // Calculate the displacement needed to resolve the collision
                float delta = 0.5f * response_coef * (dist - min_dist);

                // Adjust positions
                obj1->position.x -= n.x * (mass_ratio_2 * delta);
                obj1->position.y -= n.y * (mass_ratio_2 * delta);
                obj2->position.x += n.x * (mass_ratio_1 * delta);
                obj2->position.y += n.y * (mass_ratio_1 * delta);
            }
        }
    }
}

void applyConstraint(Solver *solver) {
    for (size_t i = 0; i < solver->object_count; i++) {
        VerletObject *obj = &solver->objects[i];

        // Vector from object to constraint center
        Vector2 v = {obj->position.x - solver->constraint_center.x, obj->position.y - solver->constraint_center.y};
        float dist = sqrtf(v.x * v.x + v.y * v.y);

        // Check if the object is outside the constraint circle
        if (dist > (solver->constraint_radius - obj->radius)) {
            // Normalize the vector
            Vector2 n = {v.x / dist, v.y / dist};

            // Move the object back to the circle's boundary
            obj->position.x = solver->constraint_center.x + n.x * (solver->constraint_radius - obj->radius);
            obj->position.y = solver->constraint_center.y + n.y * (solver->constraint_radius - obj->radius);
        }
    }
}

// Updates the simulation state
void simuUpdate(Solver *solver) {
    solver->time += solver->frame_dt;
    const float step_dt = getStepDt(solver);
    for (unsigned int i = 0; i < solver->sub_steps; i++) {
        applyGravity(solver);
        checkCollisions(solver, step_dt);
        applyConstraint(solver);
        updateObjects(solver, step_dt);
    }
}

void WriteSimulationData(FILE *file, float time, Solver *solver){
    for (size_t i = 0; i < solver->object_count; i++) {
        VerletObject *obj = &solver->objects[i];
        fprintf(file, "%.2f, RainbowParticles, %zu, %.2f, %.2f\n", time, i, obj->position.x, obj->position.y);
    }
}


int main() {
    // Initialize a Solver
    Solver solver;
    solver.time = 0.0f;
    solver.frame_dt = 1.0f / 5.0f; // 5 FPS
    solver.sub_steps = 1;
    solver.gravity = (Vector2){0.0f, -9.8f}; // Gravity

    // Assign the object to the solver
    solver.object_count = 1;
    solver.objects = malloc(solver.object_count * sizeof(VerletObject));

    for (size_t i = 0; i < solver.object_count; i++) {
        solver.objects[i].position = (Vector2){10.0f + i * 10.0f, 20.0f + i * 10.0f};
        solver.objects[i].position_last = solver.objects[i].position;
        solver.objects[i].acceleration = (Vector2){0.0f, 0.0f};
        solver.objects[i].radius = 5.0f;
        solver.objects[i].color = 0xFFFFFF; // White color in hexadecimal

        // Assign functions to function pointers
        solver.objects[i].update = objectUpdate;
        solver.objects[i].accelerate = accelerate;
        solver.objects[i].getVelocity = getVelocity;
    }


    // Opening file
    FILE *file = fopen("simulation_data.csv", "w");
    if (!file) {
        printf("Error opening file!\n");
        exit(1);
    }
    fprintf(file, "Time,Object Type,Object Index,Position X,Position Y\n");

    // Run the simulation for a few steps
    printf("Running simulation...\n");
    for (int i = 0; i < 50; i++) {
        simuUpdate(&solver);
        for (size_t j = 0; j < solver.object_count; j++) {
            printf("Object %zu - Time: %f, Position: (%f, %f), Last Position: (%f, %f), Acceleration: (%f, %f)\n", j, solver.time, solver.objects[j].position.x, solver.objects[j].position.y,
            solver.objects[j].position_last.x, solver.objects[j].position_last.y,
            solver.objects[j].acceleration.x, solver.objects[j].acceleration.y);

        }
        WriteSimulationData(file, solver.time, &solver);
    }

    free(solver.objects);
    return 0;
}