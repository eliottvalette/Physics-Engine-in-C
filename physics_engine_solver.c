// gcc -o physics_engine_solver_2 physics_engine_solver_2.c 
// ./physics_engine_solver_2

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SUB_STEPS 1
#define NUMBER_STEPS 200
#define NUMBER_OF_OBJECTS 300
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
    for (size_t i = 0; i < solver->object_count; i++) {
        VerletObject *obj = &solver->objects[i];
        float circle_radius = 350.0f;
        Vector2 circle_center = {0.0f, 0.0f};

        // Vector from the circle center to the object
        Vector2 to_obj;
        to_obj.x = obj->position.x - circle_center.x;
        to_obj.y = obj->position.y - circle_center.y;


        float dist = sqrt(to_obj.x * to_obj.x + to_obj.y * to_obj.y);

        // Check if the object is outside the allowed circle
        float max_distance = circle_radius - obj->radius;
        if  (dist > max_distance) {
            // Normalize the to_obj vector to get the direction
            Vector2 to_obj_norm;
            to_obj_norm.x = to_obj.x / dist;
            to_obj_norm.y = to_obj.y / dist;

            // Correct the position to be on the boundary
            obj->position.x = circle_center.x + to_obj_norm.x * max_distance;
            obj->position.y = circle_center.y + to_obj_norm.y * max_distance;            
        };

    };
};

void checkCollisions(Solver *solver, float dt){
        const float response_coef = 0.75f;
        // Iterate on all objects
        for (unsigned int i = 0; i < solver->object_count; ++i) {
            VerletObject *obj_1 = &solver->objects[i];
            // Iterate on object involved in new collision pairs
            for (unsigned int k = i + 1; k < solver->object_count; ++k) {
                VerletObject *obj_2 = &solver->objects[k];
                const Vector2 v = {obj_1->position.x - obj_2->position.x, obj_1->position.y - obj_2->position.y};
                const float        dist2    = v.x * v.x + v.y * v.y;
                const float        min_dist = obj_1->radius + obj_2->radius;
                // Check overlapping
                if (dist2 < min_dist * min_dist) {
                    const float dist = sqrt(dist2);
                    const Vector2 n  = {v.x / dist, v.y / dist};
                    const float mass_ratio_1 = obj_1->radius / (obj_1->radius + obj_2->radius);
                    const float mass_ratio_2 = obj_2->radius / (obj_1->radius + obj_2->radius);
                    const float delta        = 0.5f * response_coef * (dist - min_dist);
                    // Update positions
                    obj_1->position.x -= n.x * (mass_ratio_2 * delta);
                    obj_1->position.y -= n.y * (mass_ratio_2 * delta);

                    obj_2->position.x += n.x * (mass_ratio_1 * delta);
                    obj_2->position.y += n.y * (mass_ratio_1 * delta);
            }
        }
    }
}

// Update the simulation state
void simuUpdate(Solver *solver) {
    const float step_dt = solver->frame_dt / solver->sub_steps;

    for (unsigned int i = 0; i < solver->sub_steps; i++) {
        // Apply gravity
        applyGravity(solver);
        applyForce(solver, (Vector2){0.0f, 0.0f});
        checkCollisions(solver, step_dt);
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
        fprintf(file, "%.2f, RainbowParticles, %zu, %.2f, %.2f, %.2f,%d,%d,%d\n",
                time, i, obj->position.x, obj->position.y, obj->radius, obj->color[0], obj->color[1], obj->color[2]);
    };
};

int main() {
    // Initialize a Solver
    Solver solver;
    solver.time = 0.0f;
    solver.frame_dt = 1.0f / 10.0f; // FPS
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
        solver.objects[i].position = (Vector2){-200.0f + arc4random_uniform(400), -200.0f + arc4random_uniform(400)};
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
