#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define NUM_PARTICLES 2
#define NUM_RIGID_BODIES 1

// Two dimensional vector.
typedef struct {
    float x;
    float y;
} Vector2;

// Two dimensional particle.
typedef struct {
    Vector2 position;
    Vector2 velocity;
    float mass;
} Particle;

// 2D box shape
typedef struct {
    float width;
    float height;
    float mass;
    float momentOfInertia;
} BoxShape;

// Two dimensional rigid body
typedef struct {
    Vector2 position;
    Vector2 linearVelocity;
    float angle;
    float angularVelocity;
    Vector2 force;
    float torque;
    BoxShape shape;
} RigidBody;

// Global arrays of particles and rigid bodies.
Particle particles[NUM_PARTICLES];
RigidBody rigidBodies[NUM_RIGID_BODIES];

void InitializeParticles() {
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = (Vector2){arc4random_uniform(50), arc4random_uniform(50)};
        particles[i].velocity = (Vector2){0, 0};
        particles[i].mass = 1;
    }
}

Vector2 ComputeForce(Particle *particle) {
    return (Vector2){0, particle->mass * -9.81};
}

void CalculateBoxInertia(BoxShape *boxShape) {
    float m = boxShape->mass;
    float w = boxShape->width;
    float h = boxShape->height;
    boxShape->momentOfInertia = m * (w * w + h * h) / 12;
}

void InitializeRigidBodies() {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody *rigidBody = &rigidBodies[i];
        rigidBody->position = (Vector2){arc4random_uniform(50), arc4random_uniform(50)};
        rigidBody->angle = arc4random_uniform(360)/360.f * M_PI * 2;
        rigidBody->linearVelocity = (Vector2){0, 0};
        rigidBody->angularVelocity = 0;
        
        BoxShape shape;
        shape.mass = 10;
        shape.width = 1 + arc4random_uniform(2);
        shape.height = 1 + arc4random_uniform(2);
        CalculateBoxInertia(&shape);
        rigidBody->shape = shape;
    }
}

void ComputeForceAndTorque(RigidBody *rigidBody) {
    Vector2 f = (Vector2){0, 100};
    rigidBody->force = f;
    Vector2 r = (Vector2){rigidBody->shape.width / 2, rigidBody->shape.height / 2};
    rigidBody->torque = r.x * f.y - r.y * f.x;
}

void WriteSimulationData(FILE *file, float time, int simu_index){
    if (simu_index == 0){
        for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle *particle = &particles[i];
        fprintf(file, "%.2f,Particle,%d,%.2f,%.2f,\n", time, i, particle->position.x, particle->position.y);
    }

    } else if (simu_index == 1){
        for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
            RigidBody *rigidBody = &rigidBodies[i];
            fprintf(file, "%.2f,RigidBody,%d,%.2f,%.2f,%.2f\n", time, i, rigidBody->position.x, rigidBody->position.y, rigidBody->angle);
        }
    }
}

void RunSimulation(int simu_index) {
    FILE *file = fopen("simulation_data.csv", "w");
    if (!file) {
        printf("Error opening file!\n");
        exit(1);
    }

    fprintf(file, "Time,Object Type,Object ID,Position X,Position Y,Angle\n");

    float totalSimulationTime = 10; // The simulation will run for 10 seconds.
    float currentTime = 0; // This accumulates the time that has passed.
    float dt = 1; // Each step will take one second.
    
    InitializeParticles();
    InitializeRigidBodies();
    
    while (currentTime < totalSimulationTime) {
        sleep(dt);

        if (simu_index == 0) {
            for (int i = 0; i < NUM_PARTICLES; ++i) {
                Particle *particle = &particles[i];
                Vector2 force = ComputeForce(particle);
                Vector2 acceleration = (Vector2){force.x / particle->mass, force.y / particle->mass};
                particle->velocity.x += acceleration.x * dt;
                particle->velocity.y += acceleration.y * dt;
                particle->position.x += particle->velocity.x * dt;
                particle->position.y += particle->velocity.y * dt;
            }
        } else if (simu_index==1){
            for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
                RigidBody *rigidBody = &rigidBodies[i];
                ComputeForceAndTorque(rigidBody);
                Vector2 linearAcceleration = (Vector2){rigidBody->force.x / rigidBody->shape.mass, rigidBody->force.y / rigidBody->shape.mass};
                rigidBody->linearVelocity.x += linearAcceleration.x * dt;
                rigidBody->linearVelocity.y += linearAcceleration.y * dt;
                rigidBody->position.x += rigidBody->linearVelocity.x * dt;
                rigidBody->position.y += rigidBody->linearVelocity.y * dt;
                float angularAcceleration = rigidBody->torque / rigidBody->shape.momentOfInertia;
                rigidBody->angularVelocity += angularAcceleration * dt;
                rigidBody->angle += rigidBody->angularVelocity * dt;
            }
        }

        WriteSimulationData(file, currentTime, simu_index);
        currentTime += dt;
    }

    fclose(file);
}

int main() {
    printf("Simulating Particles or RigidBodies ?\n");
    char simu_object[20];
    int simu_index = -1;
    scanf("%s", simu_object);
    do {
        if (strcmp(simu_object, "Particles") == 0) {
            simu_index = 0;
        } else if (strcmp(simu_object, "RigidBodies") == 0) {
            simu_index = 1;
        };
    } while (simu_index == -1);
    printf("Running simulation...\n");
    RunSimulation(simu_index);
    return 0;
}
