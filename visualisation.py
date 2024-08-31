import pygame
import random as rd
import pandas as pd
import numpy as np
import time

# Constants for the Pygame window
WINDOW_WIDTH  = 800
WINDOW_HEIGHT = 800
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE  = (0, 0, 255)
RED   = (255, 0, 0)
FPS   = 600.0  # Frames per second

# Load the CSV file into a DataFrame
simulation_df = pd.read_csv("simulation_data.csv")
nb_steps = len(np.unique(simulation_df["Time"]))
print(f'There will be a total of {nb_steps} steps')


# Dictionary to store the trail positions for each object
trails = {}

def draw_rods(screen, x1, y1, x2, y2, color):
    pygame.draw.line(screen, color, (int(x1), int(WINDOW_HEIGHT - y1)), (int(x2), int(WINDOW_HEIGHT - y2)), 2)

def draw_particle(screen, x, y, radius=5, color=None):
    if color is None:
        color = (rd.randint(0, 255), rd.randint(0, 255), rd.randint(0, 255))
    pygame.draw.circle(screen, color, (int(x), int(WINDOW_HEIGHT - y)), radius)

def draw_rigid_body(screen, x, y, angle, width, height):
    """Draw a rigid body as a rotated rectangle."""
    points = [
        (x + width / 2, y + height / 2),
        (x + width / 2, y - height / 2),
        (x - width / 2, y - height / 2),
        (x - width / 2, y + height / 2)
    ]
    rotated_points = [(x + (px - x) * np.cos(angle) - (py - y) * np.sin(angle),
                       y + (px - x) * np.sin(angle) + (py - y) * np.cos(angle)) for px, py in points]
    pygame.draw.polygon(screen, RED, rotated_points)

def center_scene(value):
    """Translate a coordinate to be centered in the window."""
    return value + WINDOW_HEIGHT // 2

def process_pendulum(screen, pendulum_data):
    """Draw rods for the pendulum."""
    rod_color = WHITE
    indices = pendulum_data['Object Index'].values
    base_x, base_y = center_scene(0), center_scene(0)

    # Draw a square anchor in the middle of the screen
    anchor_size = 20
    anchor_color = WHITE
    pygame.draw.rect(screen, anchor_color, (base_x - anchor_size // 2, base_y - anchor_size // 2, anchor_size, anchor_size))

    for i in range(len(indices)):
        x1, y1 = (base_x, base_y) if i == 0 else (center_scene(pendulum_data.iloc[i - 1]['Position X']),
                                                  center_scene(pendulum_data.iloc[i - 1]['Position Y']))
        x2, y2 = center_scene(pendulum_data.iloc[i]['Position X']), center_scene(pendulum_data.iloc[i]['Position Y'])
        draw_rods(screen, x1, y1, x2, y2, rod_color)

        # Update trails
        object_index = pendulum_data.iloc[i]['Object Index']
        if object_index not in trails:
            trails[object_index] = []
        trails[object_index].append((x2, y2))

        # Limit the length of the trail
        if len(trails[object_index]) > nb_steps // 10:  # Adjust trail length as needed
            trails[object_index].pop(0)

def draw_trails(screen, pendulum_data):
    """Draw the trails for each node."""
    for idx, trail in enumerate(trails.values()):
        if len(trail) > 1:
            trail_color = pendulum_data.iloc[idx]['Color R'], pendulum_data.iloc[idx]['Color G'], pendulum_data.iloc[idx]['Color B']
            pygame.draw.lines(screen, trail_color, False, [(int(x), int(WINDOW_HEIGHT - y)) for x, y in trail], 1)

def draw_objects(screen, current_frame_data):
    """Draw all objects in the current frame."""
    for _, data in current_frame_data.iterrows():
        x = center_scene(data['Position X'])
        y = center_scene(data['Position Y'])
        radius = data.get('Radius', 5)
        color = (data['Color R'], data['Color G'], data['Color B'])
        
        if data['Object Type'] == "Particle":
            draw_particle(screen, x, y)
        elif data['Object Type'] == "RigidBody":
            angle = data['Angle']
            width = data['Width']
            height = data['Height']
            draw_rigid_body(screen, x, y, angle, width, height)
        elif data['Object Type'] in [" RainbowParticles", " Pendulum"]:
            draw_particle(screen, x, y, radius, color)

def run_visualization(simulation_df):
    """Run the visualization loop for the simulation."""
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Physics Engine Simulation")
    clock = pygame.time.Clock()

    time_values = simulation_df['Time'].unique()
    dt = np.diff(time_values)[0] if len(time_values) > 1 else 1
    current_time = time_values[0]
    running = True

    while running:
        screen.fill(WHITE)
        if simulation_df['Object Type'][0] in [" RainbowParticles", " Pendulum"]:
            pygame.draw.circle(screen, BLACK, (center_scene(0), center_scene(0)), 350)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        current_frame_data = simulation_df[simulation_df['Time'] == current_time]

        pendulum_data = current_frame_data[current_frame_data['Object Type'] == " Pendulum"]
        if not pendulum_data.empty:
            process_pendulum(screen, pendulum_data)
            draw_trails(screen, pendulum_data)
        draw_objects(screen, current_frame_data)

        pygame.display.flip()
        clock.tick(FPS)

        next_time_index = np.searchsorted(time_values, current_time + dt)
        if next_time_index >= len(time_values):
            current_time = time_values[0]
            time.sleep(1)
        else:
            current_time = time_values[next_time_index]

    pygame.quit()

if __name__ == "__main__":
    run_visualization(simulation_df)
fix