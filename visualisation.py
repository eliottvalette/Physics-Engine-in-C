import pygame
import random as rd
import pandas as pd
import numpy as np
import time

# Constants for the Pygame window
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
FPS = 5.0  # Frames per second

# Load the CSV file into a DataFrame
simulation_df = pd.read_csv("simulation_data.csv")
print('There will be a total of', len(np.unique(simulation_df['Time'])), 'steps')

def draw_particle(screen, x, y):
    pygame.draw.circle(screen, (rd.randint(0, 255), rd.randint(0, 255), rd.randint(0, 255)), (int(x), int(y)), 5)

def draw_rigid_body(screen, x, y, angle, width, height):
    points = [
        (x + width/2, y + height/2),
        (x + width/2, y - height/2),
        (x - width/2, y - height/2),
        (x - width/2, y + height/2)
    ]
    rotated_points = []
    for point in points:
        rotated_x = x + (point[0] - x) * np.cos(angle) - (point[1] - y) * np.sin(angle)
        rotated_y = y + (point[0] - x) * np.sin(angle) + (point[1] - y) * np.cos(angle)
        rotated_points.append((rotated_x, rotated_y))
    pygame.draw.polygon(screen, RED, rotated_points)

def get_max_min(df):
    max_x = df['Position X'].max()
    min_x = df['Position X'].min()
    max_y = df['Position Y'].max()
    min_y = df['Position Y'].min()
    return max_x, min_x, max_y, min_y

def normalize_and_scale(var, min_val, max_val, is_width):
    padding = 0.1
    if max_val == min_val: # Avoid division by zero
        return WINDOW_WIDTH / 2 if is_width else WINDOW_HEIGHT / 2
    normalized = (var - min_val) / (max_val - min_val)
    if is_width:
        return padding * WINDOW_WIDTH + (1 - 2 * padding) * WINDOW_WIDTH * normalized # pad left, pad right, and scale
    else:
        return WINDOW_HEIGHT * (1 - padding) - (1 - 2 * padding) * WINDOW_HEIGHT * normalized # due to the inverted y-axis in pygame

def run_visualization(simulation_df):
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Physics Engine Simulation")
    clock = pygame.time.Clock()

    time_values = simulation_df['Time'].unique()
    dt = np.diff(time_values)[0] if len(time_values) > 1 else 1  # Set dt to 1 if there's only one time value
    max_x, min_x, max_y, min_y = get_max_min(simulation_df)

    current_time = time_values[0]
    running = True

    while running:
        screen.fill(WHITE)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Filter the DataFrame for the current time step
        current_frame_data = simulation_df[simulation_df['Time'] == current_time]

        # Draw each object in the current frame
        for _, data in current_frame_data.iterrows():
            obj_type = data['Object Type']
            x = normalize_and_scale(data['Position X'], min_x, max_x, is_width=True)
            y = normalize_and_scale(data['Position Y'], min_y, max_y, is_width=False)
            
            if obj_type == "Particle":
                draw_particle(screen, x, y)
            elif obj_type == "RigidBody":
                angle = data['Angle']
                width = data['Width']
                height = data['Height']
                draw_rigid_body(screen, x, y, angle, width, height)
            elif obj_type == " RainbowParticles":
                draw_particle(screen, x, y)


        pygame.display.flip()
        clock.tick(FPS)  # Control the frame rate

        # Move to the next time step
        next_time_index = np.searchsorted(time_values, current_time + dt)
        if next_time_index >= len(time_values):
            current_time = time_values[0]  # Restart from the beginning
            time.sleep(1)
        else:
            current_time = time_values[next_time_index] # because 0.1 + 0.2 != 0.3

    pygame.quit()

if __name__ == "__main__":
    run_visualization(simulation_df)
