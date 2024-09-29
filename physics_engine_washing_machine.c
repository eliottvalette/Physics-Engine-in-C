// gcc -o physics_engine_solver physics_engine_solver.c 
// ./physics_engine_solver

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SUB_STEPS 2
#define NUMBER_STEPS 2000
#define NUMBER_OF_OBJECTS 400
#define BALL_RADIUS 10.0f


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
    float width;
    float height;
    float angle;
    Vector2 top_right_position;
    Vector2 bottom_left_position;
} Propeller;

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

    // Damping factor
    float damping = 0.99f;
    velocity.x *= damping;
    velocity.y *= damping;

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
    const float restitution = 0.90f; // Coefficient of restitution for the bounce

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
        if (dist > max_distance) {
            // Normalize the to_obj vector to get the direction
            Vector2 to_obj_norm;
            to_obj_norm.x = to_obj.x / dist;
            to_obj_norm.y = to_obj.y / dist;

            // Correct the position to be on the boundary
            obj->position.x = circle_center.x + to_obj_norm.x * max_distance;
            obj->position.y = circle_center.y + to_obj_norm.y * max_distance;

            // Calculate velocity
            Vector2 velocity = {
                obj->position.x - obj->position_last.x,
                obj->position.y - obj->position_last.y
            };

            // Reflect velocity using the normal and apply restitution
            float vel_dot_norm = velocity.x * to_obj_norm.x + velocity.y * to_obj_norm.y;
            velocity.x -= (1 + restitution) * vel_dot_norm * to_obj_norm.x;
            velocity.y -= (1 + restitution) * vel_dot_norm * to_obj_norm.y;

            // Update last position to reflect the bounce
            obj->position_last.x = obj->position.x - velocity.x;
            obj->position_last.y = obj->position.y - velocity.y;
        }
    }
}


void checkCollisions(Solver *solver, float dt){
        const float response_coef = 0.75f;

        // Iterate on all objects
        for (unsigned int i = 0; i < solver->object_count; ++i) {
            VerletObject *obj_main = &solver->objects[i];

            // Iterate on object involved in new collision pairs (avoid checkin A - B and B - A)
            for (unsigned int k = i + 1; k < solver->object_count; ++k) {
                VerletObject *obj_neigh    = &solver->objects[k];
                const Vector2 displacement = {obj_main->position.x - obj_neigh->position.x, obj_main->position.y - obj_neigh->position.y};
                
                // Distance between both objects
                const float dist     = sqrt(displacement.x * displacement.x + displacement.y * displacement.y);
                const float min_dist = obj_main->radius + obj_neigh->radius;

                // Check overlapping
                if (dist < min_dist) {
                    const Vector2 norm = {displacement.x / dist, displacement.y / dist};
                    const float mass_ratio_1 = obj_main->radius / (obj_main->radius + obj_neigh->radius);
                    const float mass_ratio_2 = obj_neigh->radius / (obj_main->radius + obj_neigh->radius);

                    // Compute the impulse to resolve the collision (downscaled by 0.5 and the response_coef)
                    const float delta = 0.5f * response_coef * (dist - min_dist);
                    
                    // Update positions along the displacement vector by the impulse
                    obj_main->position.x -= norm.x * (mass_ratio_2 * delta);
                    obj_main->position.y -= norm.y * (mass_ratio_2 * delta);

                    obj_neigh->position.x += norm.x * (mass_ratio_1 * delta);
                    obj_neigh->position.y += norm.y * (mass_ratio_1 * delta);
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
        // Apply variable wind
        Vector2 wind_force = {0.0f, 0.0f}; // tune as you wish with 10.0f * sin(solver->time / 3)
        applyForce(solver, wind_force);
        // Check collisions
        checkCollisions(solver, step_dt);
        // Apply constraint
        apply_constraint(solver);

        // Update all objects
        for (size_t j = 0; j < solver->object_count; j++) {
            objectUpdate(&solver->objects[j], step_dt);
        };
    };
    solver->time += solver->frame_dt;
};

// Write simulation data to a CSV file
void WriteSimulationData(FILE *file, float time, Solver *solver) {
    for (size_t i = 0; i < solver->object_count; i++) {
        VerletObject *obj = &solver->objects[i];
        fprintf(file, "%.2f, RainbowParticles, %zu, %.2f, %.2f, %.2f,%d,%d,%d", // particle index, x, y, radius, r, g, b 
                time, i, obj->position.x, obj->position.y, obj->radius, obj->color[0], obj->color[1], obj->color[2]);
        fprintf(file, "%.2f, %.2f,%.2f, %.2f, %.2f, %.2f, %.2f\n");
    };
}

int main() {
    // Initialize a Solver
    Solver solver;
    solver.time = 0.0f;
    solver.frame_dt = 1.0f / 2.0f; // FPS
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
        solver.objects[i].position = (Vector2){-200.0f + (rand() % 400), -200.0f + (rand() % 400)};
        solver.objects[i].position_last = solver.objects[i].position;
        solver.objects[i].acceleration = (Vector2){0.0f, 0.0f};
        solver.objects[i].radius = BALL_RADIUS;
        solver.objects[i].color[0] = rand() % 256;
        solver.objects[i].color[1] = rand() % 256;
        solver.objects[i].color[2] = rand() % 256;
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
