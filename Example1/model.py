import numpy as np
import random
import time
from mesa.model import Model
from mesa.agent import Agent
from mesa.space import SingleGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector


"""
    TO DO: 
     - Hacer que los robots se rodeen si van a colisionar (no intercambiar posiciones)
     - Hacer que el EPackage entregue paquetes
     - Agregar ids a los paquetes
     - Ordenar paquetes en sus respectivos estantes
     - Hacer que los robots se carguen cuando tengan poca bateria
     - Hacer que los cargadores funcionen
     - Hacer que la mitad de los robots sean de entrega y otros de acomodado
"""


class Shelf(Agent):
    def __init__(self, unique_id, model, capacity=3):
        super().__init__(unique_id, model)
        self.capacity = capacity

    def has_capacity(self):
        return self.capacity > 0

    def decrement_capacity(self):
        if self.capacity > 0:
            self.capacity -= 1

class TaskManager(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.tasks = []  # Lista de tareas pendientes
        self.robot_statuses = {}
        self.battery = 100;

    def step(self):
        # Asignar tareas a los robots libres
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot):
                if self.tasks:
                    task = self.tasks.pop(0)
                    bot.getTasks(task)
                    # self.robot_statuses[bot.unique_id] = "Busy"

    def add_task(self, task):
        self.tasks.append(task)

    def report_completion(self, robot_id):
        self.robot_statuses[robot_id] = "Free"

class EPackage(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.next_goal_time = self.model.schedule.time + random.uniform(2, 5)

    def step(self):
        current_time = self.model.schedule.time
        if current_time >= self.next_goal_time:
            # Place a Goal at this EPackage's position
            goal = Goal(self.model.next_id(), self.model)
            self.model.grid.place_agent(goal, self.pos)
            self.model.schedule.add(goal)
            
            # Set the next time a goal will appear
            self.next_goal_time = current_time + random.uniform(2, 5)


class SPackage(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Charger(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Wall(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Goal(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Bot(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.luck = np.random.uniform(0.2, 1.0)
        self.movements = 0
        self.battery = 100
        self.path = []
        self.carry = False
        self.tasks = []
        self.goal = None

    def getTasks(self, task):
        self.tasks.append(task)

    def euclidean_heuristic(self, pos, goal):
        return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)

    def at_box(self):
        return False

    def at_exit(self):
        return True

    def find_exit(self):
        for pos in self.model.grid.coord_iter():
            _, (x, y) = pos
            contents = self.model.grid.get_cell_list_contents([x, y])
            for obj in contents:
                if isinstance(obj, SPackage):
                    return (x, y)

    def a_star(self, start, goal):
        open_list = []
        closed_list = set()

        open_list.append((0 + self.euclidean_heuristic(start, goal), 0, start, None))

        while open_list:
            open_list.sort(key=lambda x: x[0])
            f, g, current, parent = open_list.pop(0)

            if current == goal:
                path = []
                while parent is not None:
                    path.append(current)
                    current = parent
                    parent = next((p for f, g, c, p in closed_list if c == current), None)
                path.reverse()
                return path

            closed_list.add((f, g, current, parent))

            for neighbor in self.model.grid.iter_neighborhood(current, moore=False, include_center=False):
                if self.model.grid.is_cell_empty(neighbor) or neighbor == goal:
                    g_new = g + 1
                    h_new = self.euclidean_heuristic(neighbor, goal)
                    f_new = g_new + h_new

                    if any(neighbor == c for f, g, c, p in closed_list):
                        continue

                    existing_node = next((i for i, (f, g, c, p) in enumerate(open_list) if c == neighbor), None)
                    if existing_node is not None:
                        if open_list[existing_node][0] > f_new:
                            open_list[existing_node] = (f_new, g_new, neighbor, current)
                    else:
                        open_list.append((f_new, g_new, neighbor, current))

        return []

    def detect_collision(self, next_pos):
        for agent in self.model.grid.get_neighbors(self.pos, moore=False, include_center=False, radius=1):
            if isinstance(agent, Bot) and agent.next_pos == next_pos:
                return True
        return False

    def avoid_collision(self):
        for neighbor in self.model.grid.iter_neighborhood(self.pos, moore=False, include_center=False):
            if self.model.grid.is_cell_empty(neighbor):
                return neighbor
        return self.pos

    def step(self):
        if self.battery > 0 and self.path and self.goal:
            self.next_pos = self.path.pop(0)

            if self.detect_collision(self.next_pos):
                self.next_pos = self.avoid_collision()
                if self.next_pos == self.pos:
                    return

            self.movements += 1
            self.battery -= 1
            if not self.carry:
                box_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, Goal)]
                if box_in_next_pos:
                    goal = box_in_next_pos[0]
                    self.model.grid.remove_agent(goal)
                    self.carry = True
                    self.goal = self.find_exit()
                    self.path = self.a_star(self.pos, self.goal)

            if self.carry:
                exit_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, SPackage)]
                if exit_in_next_pos:
                    # Interact with the SPackage without removing it
                    # Example interaction: stop moving and "drop off" the carried item
                    self.carry = False
                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)
                        if self.path:
                            self.next_pos = self.path.pop(0)
                            self.model.grid.move_agent(self, self.next_pos)
                            self.movements += 1
                            self.battery -= 1
                else:
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.model.grid.move_agent(self, self.next_pos)

        elif self.battery > 0 and self.goal is None and not self.path:
            self.goal = self.tasks.pop()[0]
            self.path = self.a_star(self.pos, self.goal)
            if self.path:
                self.next_pos = self.path.pop(0)
                self.movements += 1
                self.battery -= 1
                self.model.grid.move_agent(self, self.next_pos)

class Environment(Model):
    def __init__(self, M: int, N: int, num_agents: int = 5, num_goals: int = 1, obstacle_portion: float = 0.3, mode_start_pos='Random'):
        super().__init__()
        self.num_agents = num_agents
        self.num_goals = num_goals
        self.grid = SingleGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)
        self.num_goals_slider = num_goals  
        self.mode_start_pos = mode_start_pos
        self.special_cell = []

        self.central_system = TaskManager(0, self)
        self.schedule.add(self.central_system)

        self.desc = [
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
        'WDDDFFFFFFFFFFFFFFFFFFFFFFFFFFFFEEEFFW',
        'WFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFSSFFFSSFFFSSFFFSSFFFSSFFFSSFFFSSFFW',
        'WFFSSFFFSSFFFSSFFFSSFFFSSFFFSSFFFSSFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WCFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFGGGGFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WCFFFFFFDFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WCFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
    ]

        
        self._manual_placement(self.desc)

        self.datacollector = DataCollector(
            model_reporters={
                "MinBattery": lambda m: np.min([a.battery for a in m.schedule.agents if isinstance(a, Bot)]).astype(float),
                "MaxBattery": lambda m: np.max([a.battery for a in m.schedule.agents if isinstance(a, Bot)]).astype(float),
                "MeanBattery": lambda m: np.mean([a.battery for a in m.schedule.agents if isinstance(a, Bot)]).astype(float),
            }
        )

    def _manual_placement(self, desc):
        goalPos = (0, 0)
        box_position = []

        N = self.grid.height  # Rows
        M = self.grid.width  # Columns

        for pos in self.grid.coord_iter():
            _, (x, y) = pos
            # Ensure y is within bounds of desc
            if y < len(desc) and x < len(desc[y]):
                if desc[y][x] == 'S':  
                    shelf = Shelf(self.next_id(), self, capacity=3)
                    self.grid.place_agent(shelf, (x, y))
                elif desc[y][x] == 'W':  
                    wall = Wall(self.next_id(), self)
                    self.grid.place_agent(wall, (x, y))
                elif desc[y][x] == 'G':  
                    goal = (x, y)
                    goall = Goal(self.next_id(), self)
                    self.special_cell.append(goal)
                    box_position.append((x, y))
                    self.grid.place_agent(goall, (x, y))
                elif desc[y][x] == 'C':  
                    charger = Charger(self.next_id(), self)
                    self.grid.place_agent(charger, (x, y))
                elif desc[y][x] == 'E':  
                    entrada = EPackage(self.next_id(), self)
                    self.grid.place_agent(entrada, (x, y))
                elif desc[y][x] == 'D':  
                    salida = SPackage(self.next_id(), self)
                    self.grid.place_agent(salida, (x, y))
                elif desc[y][x] == 'R':  
                    bot = Bot(self.next_id(), self)
                    self.grid.place_agent(bot, (x, y))
                    self.schedule.add(bot)

        for box_pos in box_position:
            self.central_system.add_task((box_pos, goalPos))


    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()
        self.running = any([a.battery > 0 for a in self.schedule.agents if isinstance(a, Bot)])

