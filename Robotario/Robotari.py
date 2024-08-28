import numpy as np
import matplotlib.pyplot as plt

class QLearningAgent:
    def __init__(self, width, height, resolution, action_size, alpha=0.1, gamma=0.9, epsilon=0.5, epsilon_decay=0.99, epsilon_min=0.01):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.action_size = action_size
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.q_table = np.zeros((int(width / resolution), int(height / resolution), action_size))
        self.visited_states = set()

    def choose_action(self, state_index):
        if np.random.rand() <= self.epsilon:
            return np.random.choice(self.action_size)  # Exploración
        return np.argmax(self.q_table[state_index])  # Explotación

    def update_q_value(self, state_index, action, reward, next_state_index):
        best_next_action = np.argmax(self.q_table[next_state_index])
        td_target = reward + self.gamma * self.q_table[next_state_index][best_next_action]
        td_error = td_target - self.q_table[state_index][action]
        self.q_table[state_index][action] += self.alpha * td_error

    def decay_epsilon(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def state_to_index(self, state):
        x, y = state
        x_index = int((x + (self.width / 2)) / self.resolution)
        y_index = int((y + (self.height / 2)) / self.resolution)
        return (x_index, y_index)

    def add_visited_state(self, state_index):
        self.visited_states.add(state_index)

    def has_visited(self, state_index):
        return state_index in self.visited_states

def load_obstacle(filename):
    data = np.loadtxt(filename, delimiter=',')
    x_coords = data[0, :]  # Primera fila es x
    y_coords = data[1, :]  # Segunda fila es y
    return list(zip(x_coords, y_coords))

class ComplexEnvironment:
    def __init__(self, width, height, robot_radius, goal_states, obstacles=[]):
        self.width = width
        self.height = height
        self.robot_radius = robot_radius
        self.goal_states = goal_states
        self.obstacles = obstacles
        self.state = None

    def reset(self):
        self.state = (0, -2)  # Iniciar en la segunda posición inicial
        return self.state

    def is_obstacle(self, pos):
        for obs in self.obstacles:
            obs = np.array(obs)
            x_min, x_max = np.min(obs[:, 0]), np.max(obs[:, 0])
            y_min, y_max = np.min(obs[:, 1]), np.max(obs[:, 1])
            if x_min <= pos[0] <= x_max and y_min <= pos[1] <= y_max:
                return True
        return False

    def step(self, action, agent):
        x, y = self.state
        if action == 0:  # Arriba
            next_state = (x, y + 0.1)
        elif action == 1:  # Abajo
            next_state = (x, y - 0.1)
        elif action == 2:  # Izquierda
            next_state = (x - 0.1, y)
        elif action == 3:  # Derecha
            next_state = (x + 0.1, y)
        else:
            next_state = self.state

        if not (-3 <= next_state[0] <= 3 and -2 <= next_state[1] <= 2) or self.is_obstacle(next_state):
            next_state = self.state

        state_index = agent.state_to_index(next_state)
        distance_to_goal = np.linalg.norm(np.array(next_state) - np.array(self.goal_states[0]))
        
        if agent.has_visited(state_index):
            reward = -10
        else:
            reward = -distance_to_goal

        if distance_to_goal < 0.2:
            reward = 100
            done = True
        else:
            done = False

        agent.add_visited_state(state_index)
        self.state = next_state
        return next_state, reward, done

def train(agent, env, episodes=1000, log_interval=50):
    all_steps = []
    for episode in range(episodes):
        state = env.reset()
        state_index = agent.state_to_index(state)
        done = False
        steps = []

        while not done:
            action = agent.choose_action(state_index)
            next_state, reward, done = env.step(action, agent)
            next_state_index = agent.state_to_index(next_state)

            agent.update_q_value(state_index, action, reward, next_state_index)
            state_index = next_state_index
            steps.append(next_state)

        agent.decay_epsilon()
        all_steps.append(steps)

        if episode % log_interval == 0:
            print(f"Episodio {episode}/{episodes}")

    return steps

# Cargar obstáculos desde los archivos
obstacle_1 = load_obstacle('CornersObs1.txt')
obstacle_2 = load_obstacle('CornersObs2.txt')
obstacle_3 = load_obstacle('CornersObs3.txt')
obstacle_4 = load_obstacle('CornersObs4.txt')

obstacles = [obstacle_1, obstacle_2, obstacle_3, obstacle_4]

# Cargar posiciones objetivo
target_positions = np.loadtxt('TargetPositions.txt', delimiter=',')
target_positions = list(zip(target_positions[0], target_positions[1]))

# Parámetros del entorno
width = 6.0  # Ancho del área de simulación (6 metros)
height = 4.0  # Alto del área de simulación (4 metros)
resolution = 0.1  # Resolución del grid (10 celdas por metro)
robot_radius = 0.08  # Radio del robot

# Crear el entorno y el agente
env = ComplexEnvironment(width, height, robot_radius, target_positions, obstacles)
agent = QLearningAgent(width, height, resolution, action_size=4)

# Entrenar al agente y obtener la secuencia de pasos
steps = train(agent, env)

# Seleccionar 20 pasos uniformemente distribuidos
if len(steps) > 20:
    selected_steps = steps[::len(steps) // 20][:20]
else:
    selected_steps = steps

# Guardar las coordenadas de la ruta en un archivo de texto
with open('robot_path.txt', 'w') as f:
    for step in selected_steps:
        f.write(f"{step[0]},{step[1]}\n")

# Simulación del movimiento del robot
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([-3, 3])  # Ancho de -3 a 3 metros (total 6 metros)
ax.set_ylim([-2, 2])  # Alto de -2 a 2 metros (total 4 metros)

# Dibujar obstáculos
for obs in obstacles:
    obs = np.array(obs)
    ax.fill(obs[:, 0], obs[:, 1], 'r')

# Dibujar la ruta del robot
steps = np.array(steps)
ax.plot(steps[:, 0], steps[:, 1], 'b--', marker='o')

# Dibujar posiciones inicial y objetivo
ax.plot(0, -2, 'go', markersize=10, label='Inicio')
for target in target_positions:
    ax.plot(target[0], target[1], 'gx', markersize=10, label='Objetivo')

plt.legend()
plt.title('Simulación del movimiento del robot')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.show()

print("Ruta guardada en 'robot_path.txt'.")
