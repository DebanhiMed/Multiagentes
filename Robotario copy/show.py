import numpy as np
import matplotlib.pyplot as plt

def load_obstacle(filename):
    data = np.loadtxt(filename, delimiter=',')
    x_coords = data[0, :]
    y_coords = data[1, :]
    return list(zip(x_coords, y_coords))

def expand_obstacle(obstacle, expansion=0.1):
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

def load_path(filename):
    data = np.loadtxt(filename, delimiter=',')
    return list(zip(data[:, 0], data[:, 1]))

def round_pos(pos, decimals=2):
    if isinstance(pos, (list, tuple)) and isinstance(pos[0], (list, tuple)):
        return [tuple(round(coord, decimals) for coord in p) for p in pos]
    return tuple(round(coord, decimals) for coord in pos)

def calculate_vector(p1, p2):
    return np.array(p2) - np.array(p1)

def vector_angle(v1, v2):
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    if norm_v1 == 0 or norm_v2 == 0:
        return 0
    cos_angle = dot_product / (norm_v1 * norm_v2)
    return np.arccos(np.clip(cos_angle, -1.0, 1.0))

def direction_change_detected(p1, p2, p3, angle_threshold=np.radians(10)):
    vec1 = calculate_vector(p1, p2)
    vec2 = calculate_vector(p2, p3)
    angle = vector_angle(vec1, vec2)
    return angle > angle_threshold

def plot_path_with_direction_changes(ax, path, label, color, marker):
    if len(path) < 3:
        return

    # Plot the path
    ax.plot([p[0] for p in path], [p[1] for p in path], color + '--', marker=marker, label=label)

    # Detect direction changes
    for i in range(1, len(path) - 1):
        prev_point = np.array(path[i - 1])
        curr_point = np.array(path[i])
        next_point = np.array(path[i + 1])

        if direction_change_detected(prev_point, curr_point, next_point):
            ax.plot(curr_point[0], curr_point[1], 'ko', markersize=8)  # Marker for direction change
            ax.text(curr_point[0], curr_point[1], f'({curr_point[0]:.2f}, {curr_point[1]:.2f})', fontsize=8, ha='right')

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
initial_positions = load_path('InitialPositions.txt')
robot1_initial_position, robot2_initial_position = initial_positions

# Cargar rutas de los robots desde archivos
robot1_path = load_path('robot_path1.txt')
robot2_path = load_path('robot_path2.txt')

# Parámetros del entorno
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([-3, 3])
ax.set_ylim([-2, 2])

# Dibujar obstáculos
for obs in obstacles:
    expanded_obs = expand_obstacle(obs)
    expanded_obs = np.array(expanded_obs)
    ax.fill(expanded_obs[:, 0], expanded_obs[:, 1], 'r', alpha=0.5)

# Dibujar la ruta de ambos robots y los cambios de dirección
plot_path_with_direction_changes(ax, robot1_path, 'Robot 1', 'b', 'o')
plot_path_with_direction_changes(ax, robot2_path, 'Robot 2', 'g', 'x')

# Dibujar posiciones iniciales y objetivos
ax.plot(robot1_initial_position[0], robot1_initial_position[1], 'go', markersize=10, label='Inicio Robot 1')
ax.plot(robot2_initial_position[0], robot2_initial_position[1], 'mo', markersize=10, label='Inicio Robot 2')

for target in target_positions:
    ax.plot(target[0], target[1], 'kx', markersize=10, label='Objetivo')

plt.legend()
plt.title('Obstáculos, Puntos y Rutas Generadas con Cambios de Dirección')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.show()
