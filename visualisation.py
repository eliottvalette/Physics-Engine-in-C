import pandas as pd
import matplotlib.pyplot as plt

def visualize_simulation(csv_file):
    data = pd.read_csv(csv_file)

    for time in data['Time'].unique():
        plt.figure()
        plt.title(f'Time: {time:.2f} seconds')
        
        # Filter data for the current time
        time_data = data[data['Time'] == time]

        # Plot particles
        particles = time_data[time_data['Object Type'] == 'Particle']
        plt.plot(particles['Position X'], particles['Position Y'], 'bo', label='Particles')

        # Plot rigid bodies
        rigid_bodies = time_data[time_data['Object Type'] == 'RigidBody']
        plt.plot(rigid_bodies['Position X'], rigid_bodies['Position Y'], 'ro', label='RigidBodies')

        plt.xlim(0, 50)
        plt.ylim(0, 50)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.legend()
        plt.show()

if __name__ == "__main__":
    visualize_simulation('simulation_data.csv')
