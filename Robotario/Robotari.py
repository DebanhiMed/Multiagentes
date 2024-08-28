import numpy as np
import matplotlib.pyplot as plt
import heapq
from matplotlib.path import Path

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

def load_initial_positions(filename):
    data = np.loadtxt(filename, delimiter=',')
    return list(zip(data[0, :], data[1, :]))

def round_pos(pos, decimals=2):
    if isinstance(pos, (list, tuple)) and isinstance(pos[0], (list, tuple)):
        return [tuple(round(coord, decimals) for coord in p) for p in pos]
    return tuple(round(coord, decimals) for coord in pos)

class ComplexEnvironment:
    def __init__(self, width, height, robot_radius, goal_states, obstacles=[]):
        self.width = width
        self.height = height
        self.robot_radius = robot_radius
        self.goal_states = [round_pos(goal) for goal in goal_states]
        self.obstacles = [round_pos(expand_obstacle(obs)) for obs in obstacles]
        self.state = None

    def reset(self, initial_state):
        self.state = round_pos(initial_state)
        return self.state

    def is_obstacle(self, pos):
        pos = np.array(round_pos(pos))
        for obs in self.obstacles:
            polygon = Path(np.array(obs))
            if polygon.contains_point(pos):
                return True
        return False

    def neighbors(self, pos):
        x, y = pos
        steps = [(x + 0.05, y), (x - 0.05, y), (x, y + 0.05), (x, y - 0.05)]
        return [round_pos(step) for step in steps if self.is_within_bounds(step) and not self.is_obstacle(step)]

    def is_within_bounds(self, pos):
        x, y = pos
        return -3 <= x <= 3 and -2 <= y <= 2

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def a_star_search(env, start, goal):
    start = round_pos(start)
    goal = round_pos(goal)

    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        current = heapq.heappop(frontier)[1]

        if current == goal:
            break

        for next in env.neighbors(current):
            new_cost = cost_so_far[current] + heuristic(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                heapq.heappush(frontier, (priority, next))
                came_from[next] = current

    if goal not in came_from:
        print(f"No se pudo encontrar un camino desde {start} hasta {goal}.")
        return []

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def execute_movement(env, initial_position, goals):
    state = initial_position
    total_path = []

    for goal in goals:
        path = a_star_search(env, state, goal)
        if not path:
            print(f"Camino no encontrado para el objetivo {goal}.")
            continue
        if total_path:
            path = path[1:]
        total_path.extend(path)
        state = goal

    return total_path

# Cargar obstáculos desde los archivos
obstacle_1 = load_obstacle('CornersObs1.txt')
obstacle_2 = load_obstacle('CornersObs2.txt')
obstacle_3 = load_obstacle('CornersObs3.txt')
obstacle_4 = load_obstacle('CornersObs4.txt')

obstacles = [obstacle_1, obstacle_2, obstacle_3, obstacle_4]

# Cargar posiciones objetivo
target_positions = np.loadtxt('TargetPositions.txt', delimiter=',')
target_positions = list(zip(target_positions[0], target_positions[1]))

# Dividir los objetivos entre los dos robots
half = len(target_positions) // 2
robot1_goals = target_positions
robot2_goals = target_positions

# Cargar posiciones iniciales de los robots
initial_positions = load_initial_positions('InitialPositions.txt')
robot1_initial_position, robot2_initial_position = initial_positions

# Parámetros del entorno
width = 6.0
height = 4.0
robot_radius = 0.08

# Crear el entorno
env = ComplexEnvironment(width, height, robot_radius, target_positions, obstacles)

# Planificar rutas para ambos robots
total_path_robot1 = execute_movement(env, robot1_initial_position, robot1_goals)
total_path_robot2 = execute_movement(env, robot2_initial_position, robot2_goals)

# Verificar si se encontraron caminos
if not total_path_robot1:
    print("Robot 1 no pudo encontrar un camino válido.")
if not total_path_robot2:
    print("Robot 2 no pudo encontrar un camino válido.")

# Guardar las coordenadas de las rutas en archivos de texto
with open('robot_path1.txt', 'w') as f1, open('robot_path2.txt', 'w') as f2:
    for step in total_path_robot1:
        f1.write(f"{step[0]},{step[1]}\n")
    for step in total_path_robot2:
        f2.write(f"{step[0]},{step[1]}\n")

# Simulación del movimiento de los robots
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([-3, 3])
ax.set_ylim([-2, 2])

# Dibujar obstáculos
for obs in obstacles:
    expanded_obs = expand_obstacle(obs)
    expanded_obs = np.array(expanded_obs)
    ax.fill(expanded_obs[:, 0], expanded_obs[:, 1], 'r')

# Dibujar la ruta de ambos robots
if total_path_robot1:
    total_path_robot1 = np.array(total_path_robot1)
    ax.plot(total_path_robot1[:, 0], total_path_robot1[:, 1], 'b--', marker='o', label='Robot 1')

if total_path_robot2:
    total_path_robot2 = np.array(total_path_robot2)
    ax.plot(total_path_robot2[:, 0], total_path_robot2[:, 1], 'g--', marker='x', label='Robot 2')

# Dibujar posiciones iniciales y objetivos
ax.plot(robot1_initial_position[0], robot1_initial_position[1], 'go', markersize=10, label='Inicio Robot 1')
ax.plot(robot2_initial_position[0], robot2_initial_position[1], 'mo', markersize=10, label='Inicio Robot 2')

for target in target_positions:
    ax.plot(target[0], target[1], 'kx', markersize=10, label='Objetivo')

plt.legend()
plt.title('Simulación del Movimiento de Dos Robots con A*')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.show()

print("Rutas guardadas en 'robot_path1.txt' y 'robot_path2.txt'.")
