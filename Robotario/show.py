import numpy as np
import matplotlib.pyplot as plt

def load_obstacle(filename):
    data = np.loadtxt(filename, delimiter=',')
    x_coords = data[0, :]
    y_coords = data[1, :]
    return list(zip(x_coords, y_coords))

def expand_obstacle(obstacle, expansion=0.0):
    expanded_obstacle = []
    center_x = np.mean([point[0] for point in obstacle])
    center_y = np.mean([point[1] for point in obstacle])
    
    for x, y in obstacle:
        direction_x = x - center_x
        direction_y = y - center_y
        norm = np.sqrt(direction_x**2 + direction_y**2)
        if norm == 0:
            expanded_obstacle.append((x, y))
        else:
            expand_x = x + expansion * (direction_x / norm)
            expand_y = y + expansion * (direction_y / norm)
            expanded_obstacle.append((expand_x, expand_y))
    
    return expanded_obstacle

def load_initial_positions(filename):
    data = np.loadtxt(filename, delimiter=',')
    return list(zip(data[0, :], data[1, :]))

def plot_map(obstacles, initial_positions, target_positions):
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim([-3.5, 3.5])
    ax.set_ylim([-2.5, 2.5])

    # Dibujar obstáculos
    for obs in obstacles:
        expanded_obs = expand_obstacle(obs)
        expanded_obs = np.array(expanded_obs)
        ax.fill(expanded_obs[:, 0], expanded_obs[:, 1], 'r')

    # Dibujar posiciones iniciales
    for i, pos in enumerate(initial_positions):
        ax.plot(pos[0], pos[1], 'bo', markersize=10)
        ax.text(pos[0] + 0.05, pos[1] + 0.05, f'Inicio {i+1}: ({pos[0]}, {pos[1]})', fontsize=9)

    # Dibujar objetivos
    for i, target in enumerate(target_positions):
        ax.plot(target[0], target[1], 'gx', markersize=10)
        ax.text(target[0] + 0.05, target[1] + 0.05, f'Objetivo {i+1}: ({target[0]}, {target[1]})', fontsize=9)

    plt.title('Mapa con Obstáculos, Posiciones Iniciales y Objetivos')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()

# Cargar obstáculos desde los archivos
obstacle_1 = load_obstacle('CornersObs1.txt')
obstacle_2 = load_obstacle('CornersObs2.txt')
obstacle_3 = load_obstacle('CornersObs3.txt')
obstacle_4 = load_obstacle('CornersObs4.txt')

obstacles = [obstacle_1, obstacle_2, obstacle_3, obstacle_4]

# Cargar posiciones objetivo
target_positions = np.loadtxt('TargetPositions.txt', delimiter=',')
target_positions = list(zip(target_positions[0], target_positions[1]))

# Cargar posiciones iniciales de los robots
initial_positions = load_initial_positions('InitialPositions.txt')

# Graficar el mapa
plot_map(obstacles, initial_positions, target_positions)
