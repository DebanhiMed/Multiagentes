import numpy as np
import matplotlib.pyplot as plt

class QLearningAgent:
    def __init__(self, state_size, action_size, alpha=0.1, gamma=0.9, epsilon=1.0, epsilon_decay=0.995, epsilon_min=0.01):
        self.state_size = state_size
        self.action_size = action_size
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.q_table = np.zeros((state_size, action_size))

    def choose_action(self, state_index):
        if np.random.rand() <= self.epsilon:
            return np.random.choice(self.action_size)  # Exploración
        return np.argmax(self.q_table[state_index])  # Explotación

    def update_q_value(self, state_index, action, reward, next_state_index):
        best_next_action = np.argmax(self.q_table[next_state_index])
        td_target = reward + self.gamma * self.q_table[next_state_index, best_next_action]
        td_error = td_target - self.q_table[state_index, action]
        self.q_table[state_index, action] += self.alpha * td_error

    def decay_epsilon(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def state_to_index(self, state, width):
        x, y = state
        return int((y + 3) * width + (x + 3))  # Ajustar el offset para el índice

class ComplexEnvironment:
    def __init__(self, width, height, robot_radius, goal_states, obstacles=[]):
        self.width = width
        self.height = height
        self.robot_radius = robot_radius
        self.goal_states = goal_states
        self.obstacles = obstacles
        self.state = None
        self.goal_index = 0

    def reset(self):
        self.state = (-3.0, 0.0)  # Iniciar en la primera posición inicial
        self.goal_index = 0  # Reiniciar al primer objetivo
        return self.state

    def is_obstacle(self, pos):
        for obs in self.obstacles:
            x = obs[0]
            y = obs[1]
            if np.min(x) <= pos[0] <= np.max(x) and np.min(y) <= pos[1] <= np.max(y):
                return True
        return False

    def step(self, action):
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

        if not (-3 <= next_state[0] <= 3 and -3 <= next_state[1] <= 3) or self.is_obstacle(next_state):
            next_state = self.state

        reward = -0.1  # Penalización pequeña por moverse

        # Comprobar si se ha alcanzado el objetivo actual
        if np.linalg.norm(np.array(next_state) - np.array(self.goal_states[self.goal_index])) < 0.1:
            reward = 10
            self.goal_index += 1  # Pasar al siguiente objetivo

            if self.goal_index >= len(self.goal_states):
                done = True  # Terminar si se alcanzaron todos los objetivos
            else:
                done = False
        else:
            done = False

        self.state = next_state
        return next_state, reward, done

def train(agent, env, episodes=1000, log_interval=100):
    all_steps = []
    for episode in range(episodes):
        state = env.reset()
        state_index = agent.state_to_index(state, env.width)
        done = False
        steps = []

        while not done:
            action = agent.choose_action(state_index)
            next_state, reward, done = env.step(action)
            next_state_index = agent.state_to_index(next_state, env.width)

            agent.update_q_value(state_index, action, reward, next_state_index)
            state_index = next_state_index
            steps.append(next_state)

        agent.decay_epsilon()
        all_steps.append(steps)

        if episode % log_interval == 0:
            print(f"Episodio {episode}/{episodes}")

    return all_steps[-1]

# Coordenadas de los obstáculos (manualmente ajustadas)
obstacles = [
    ([-0.3952, -1.0048, -1.0048, -0.3952], [-0.1952, -0.1952, -0.8048, -0.8048]),  # Obstáculo 1
    ([0.3048, -0.3048, -0.3048, 0.3048], [0.3048, 0.3048, -0.3048, -0.3048]),     # Obstáculo 2
    ([0.3048, -0.3048, -0.3048, 0.3048], [1.0048, 1.0048, 0.3952, 0.3952]),       # Obstáculo 3
    ([1.5, 1.0689, 1.0689, 1.5], [1.931, 1.931, 1.0689, 1.0689])                 # Obstáculo 4
]

# Coordenadas de las posiciones iniciales (manualmente ajustadas)
initial_positions = [(-3.0, 0.0), (-1.5, -2.0)]

# Coordenadas de las posiciones objetivo (manualmente ajustadas)
target_positions = [(-1.25, 0.0), (0.9, 1.9), (-0.5, 0.0), (1.25, -0.4)]

# Parámetros del entorno
width = 6.0  # Ancho del área de simulación
height = 4.0  # Alto del área de simulación
robot_radius = 0.08  # Radio del robot

# Crear el entorno y el agente
env = ComplexEnvironment(width, height, robot_radius, target_positions, obstacles)
agent = QLearningAgent(state_size=int(width*height*10), action_size=4)

# Entrenar al agente y obtener la secuencia de pasos
steps = train(agent, env)

# Seleccionar 20 pasos uniformemente distribuidos
if len(steps) > 20:
    selected_steps = steps[::len(steps)//20][:20]
else:
    selected_steps = steps

# Guardar las coordenadas de la ruta en un archivo de texto
with open('robot_path.txt', 'w') as f:
    for step in selected_steps:
        f.write(f"{step[0]},{step[1]}\n")

# Simulación del movimiento del robot
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])

# Dibujar obstáculos
for obs in obstacles:
    ax.fill(obs[0], obs[1], 'r')

# Dibujar la ruta del robot
steps = np.array(steps)
ax.plot(steps[:, 0], steps[:, 1], 'b--', marker='o')

# Dibujar posiciones inicial y objetivo
ax.plot(initial_positions[0][0], initial_positions[0][1], 'go', markersize=10, label='Inicio')
for target in target_positions:
    ax.plot(target[0], target[1], 'gx', markersize=10, label='Objetivo')

plt.legend()
plt.title('Simulación del movimiento del robot')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.show()

print("Ruta guardada en 'robot_path.txt'.")