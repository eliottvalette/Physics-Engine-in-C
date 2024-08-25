// gcc -o physics_engine_solver_2 physics_engine_solver_2.c 
// ./physics_engine_solver_2

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SUB_STEPS 2
#define NUMBER_STEPS 20000
#define NUMBER_OF_OBJECTS 3
#define BALL_RADIUS 12.0f

// Two-dimensional vector.
typedef struct {
    float x;
    float y;
} Vector2;

// Verlet Object and its properties
typedef struct {
    Vector2 position;
    Vector2 position_last;
    Vector2 acceleration;
    float radius;
    int color[3];
} VerletObject;

typedef struct {
    float time;
    float frame_dt;
    unsigned int sub_steps;
    Vector2 gravity;
    VerletObject *objects;
    size_t object_count;
} Solver;

// Simplified update function using Verlet integration
void objectUpdate(VerletObject *obj, float dt) {
    // Calculate velocity
    Vector2 velocity = {
        obj->position.x - obj->position_last.x,
        obj->position.y - obj->position_last.y
    };

    // Save position
    obj->position_last = obj->position;

    // Update position using Verlet integration formula
    obj->position.x += velocity.x + obj->acceleration.x * (dt * dt);
    obj->position.y += velocity.y + obj->acceleration.y * (dt * dt);

    // Reset acceleration
    obj->acceleration = (Vector2){0.0f, 0.0f};
}

void applyAcceleration(Solver *solver, Vector2 a) { // Apply acceleration to all objects
    for (size_t i = 0; i < solver->object_count; i++) {
        solver->objects[i].acceleration.x += a.x;
        solver->objects[i].acceleration.y += a.y;
    }
}

void applyGravity(Solver *solver) {
    applyAcceleration(solver, solver->gravity);
}

void applyForce(Solver *solver, Vector2 force){
    applyAcceleration(solver, force);
}

void apply_constraint(Solver *solver) {
    // connect the first node to the center of the circle
    VerletObject *obj = &solver->objects[0];
    float rods_size = 300.0f / NUMBER_OF_OBJECTS;
    Vector2 pendulum_center = {0.0f, 0.0f};

    // Vector from the center of the circle
    Vector2 to_obj;
    to_obj.x = obj->position.x - pendulum_center.x;
    to_obj.y = obj->position.y - pendulum_center.y;

    float dist = sqrt(to_obj.x * to_obj.x + to_obj.y * to_obj.y);

    float max_distance = rods_size - obj->radius;
    if  (dist != max_distance) {
        // Normalize the to_obj vector to get the direction
        Vector2 to_obj_norm;
        to_obj_norm.x = to_obj.x / dist;
        to_obj_norm.y = to_obj.y / dist;

        // Correct the position to be on the boundary
        obj->position.x = pendulum_center.x + to_obj_norm.x * max_distance;
        obj->position.y = pendulum_center.y + to_obj_norm.y * max_distance;            
    };
    
    // Connect each node to the previous node
    for (size_t i = 1; i < solver->object_count; i++) {
        VerletObject *obj_prev = &solver->objects[i-1];
        VerletObject *obj_next = &solver->objects[i];

        // Vector from the previous node
        Vector2 to_obj;
        to_obj.x = obj_next->position.x - obj_prev->position.x;
        to_obj.y = obj_next->position.y - obj_prev->position.y;

        float dist = sqrt(to_obj.x * to_obj.x + to_obj.y * to_obj.y);        
        if  (dist != max_distance) {
            // Normalize the to_obj vector to get the direction
            Vector2 to_obj_norm;
            to_obj_norm.x = to_obj.x / dist;
            to_obj_norm.y = to_obj.y / dist;

            // Correct the position to be on the boundary
            obj_next->position.x = obj_prev->position.x + to_obj_norm.x * max_distance;
            obj_next->position.y = obj_prev->position.y + to_obj_norm.y * max_distance;            
        };
    };
};

// Update the simulation state
void simuUpdate(Solver *solver) {
    const float step_dt = solver->frame_dt / solver->sub_steps;

    for (unsigned int i = 0; i < solver->sub_steps; i++) {
        // Apply gravity
        applyGravity(solver);
        applyForce(solver, (Vector2){0.0f, 0.0f});
        apply_constraint(solver);

        // Update all objects
        for (size_t j = 0; j < solver->object_count; j++) {
            objectUpdate(&solver->objects[j], step_dt);
        };
    };
    solver->time += solver->frame_dt;
};

// Function to write simulation data to a CSV file
void WriteSimulationData(FILE *file, float time, Solver *solver) {
    for (size_t i = 0; i < solver->object_count; i++) {
        VerletObject *obj = &solver->objects[i];
        fprintf(file, "%.2f, Pendulum, %zu, %.2f, %.2f, %.2f,%d,%d,%d\n",
                time, i, obj->position.x, obj->position.y, obj->radius, obj->color[0], obj->color[1], obj->color[2]);
    };
};

int main() {
    // Initialize a Solver
    Solver solver;
    solver.time = 0.0f;
    solver.frame_dt = 1.0f / 60.0f; // FPS
    solver.sub_steps = SUB_STEPS;
    solver.gravity = (Vector2){0.0f, -9.8f}; // Gravity
    solver.object_count = NUMBER_OF_OBJECTS;

    // Allocate memory for objects
    solver.objects = malloc(solver.object_count * sizeof(VerletObject));
    if (!solver.objects) {
        printf("Memory allocation failed!\n");
        return 1;
    };


    for (unsigned int i = 0; i < solver.object_count; i++){
        // Initialize the object
        solver.objects[i].position = (Vector2){-1.0f, 1.0f}; // random on x and up on y
        solver.objects[i].position_last = solver.objects[i].position;
        solver.objects[i].acceleration = (Vector2){0.0f, 0.0f};
        solver.objects[i].radius = BALL_RADIUS;
        solver.objects[i].color[0] = arc4random_uniform(256);
        solver.objects[i].color[1] = arc4random_uniform(256);
        solver.objects[i].color[2] = arc4random_uniform(256);
    };

    // Open the CSV file for writing
    FILE *file = fopen("simulation_data.csv", "w");
    if (!file) {
        printf("Error opening file!\n");
        free(solver.objects);
        return 1;
    };
    fprintf(file, "Time,Object Type,Object Index,Position X,Position Y,Radius,Color R,Color G,Color B\n");

    // Run the simulation for a few steps
    for (int i = 0; i < NUMBER_STEPS; i++) {
        simuUpdate(&solver);
        // Write the current state to the CSV file
        WriteSimulationData(file, solver.time, &solver);
    }

    // Close the file and free memory
    fclose(file);
    free(solver.objects);
    return 0;
}
