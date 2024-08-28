import numpy as np
import random
from mesa import Model, Agent
from mesa.space import SingleGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

class ShelfA(Agent):
    def __init__(self, unique_id, model, capacity=3):
        super().__init__(unique_id, model)
        self.capacity = capacity

    def has_capacity(self):
        return self.capacity > 0

    def decrement_capacity(self):
        if self.capacity > 0:
            self.capacity -= 1

class ShelfB(Agent):
    def __init__(self, unique_id, model, capacity=3):
        super().__init__(unique_id, model)
        self.capacity = capacity

    def has_capacity(self):
        return self.capacity > 0

    def decrement_capacity(self):
        if self.capacity > 0:
            self.capacity -= 1

class ShelfC(Agent):
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

    def step(self):
        # Asignar tareas a los robots libres
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot):
                if self.tasks:
                    task = self.tasks.pop(0)
                    bot.getTasks(task)

    def add_task(self, task):
        self.tasks.append(task)

class PackageA(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class PackageB(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class PackageC(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class SPackage(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Wall(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Bot(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.movements = 0
        self.battery = 100  
        self.path = []
        self.carry = False
        self.tasks = []
        self.goal = None
        self.charging = False
        self.isCarryingA = False
        self.isCarryingB = False
        self.isCarryingC = False

    def getTasks(self, task):
        self.tasks.append(task)

    def euclidean_heuristic(self, pos, goal):
        return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)

    def find_exit(self):
        for pos in self.model.grid.coord_iter():
            _, (x, y) = pos
            contents = self.model.grid.get_cell_list_contents([x, y])
            for obj in contents:
                if isinstance(obj, SPackage):
                    return (x, y)

    def a_star(self, start, goal, avoid_positions=[]):
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
                    if neighbor in avoid_positions:
                        continue  # Skip this neighbor if it's in the avoid_positions list
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

    def find_alternative_path(self, start, goal):
        avoid_positions = [agent.next_pos for agent in self.model.schedule.agents if isinstance(agent, Bot) and agent.next_pos is not None]
        return self.a_star(start, goal, avoid_positions)

    def avoid_collision(self):
        for neighbor in self.model.grid.iter_neighborhood(self.pos, moore=False, include_center=False):
            if self.model.grid.is_cell_empty(neighbor):
                return neighbor
        return self.pos

    def step(self):
        # Eliminar condición de batería baja, ya que es infinita
        if self.path and self.goal:
            self.next_pos = self.path.pop(0)

            if self.detect_collision(self.next_pos):
                self.path = self.find_alternative_path(self.pos, self.goal)
                if not self.path:
                    return  # No alternative path found, wait in place
                self.next_pos = self.path.pop(0)

            self.movements += 1
            if not self.carry:
                box_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, (PackageA, PackageB, PackageC))]
                if box_in_next_pos:
                    goal = box_in_next_pos[0]
                    self.model.grid.remove_agent(goal)
                    self.carry = True
                    self.update_carrying_flags(goal)
                    self.goal = self.find_exit()
                    self.path = self.a_star(self.pos, self.goal)

            if self.carry:
                exit_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, SPackage)]
                if exit_in_next_pos:
                    self.carry = False
                    self.reset_carrying_flags()
                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)
                        if self.path:
                            self.next_pos = self.path.pop(0)
                            self.model.grid.move_agent(self, self.next_pos)
                            self.movements += 1
                            self.battery = 100
                else:
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.model.grid.move_agent(self, self.next_pos)

        elif self.goal is None and not self.path:
            if self.tasks:
                self.goal = self.tasks.pop()[0]
                self.path = self.a_star(self.pos, self.goal)
                if self.path:
                    self.next_pos = self.path.pop(0)
                    self.movements += 1
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.find_new_task()

    def find_new_task(self):
        # Buscar una nueva tarea después de completar una
        if self.model.central_system.tasks:
            self.goal = self.model.central_system.tasks.pop(0)[0]
            self.path = self.a_star(self.pos, self.goal)

    def update_carrying_flags(self, package):
        if isinstance(package, PackageA):
            self.isCarryingA = True
        elif isinstance(package, PackageB):
            self.isCarryingB = True
        elif isinstance(package, PackageC):
            self.isCarryingC = True

    def reset_carrying_flags(self):
        self.isCarryingA = False
        self.isCarryingB = False
        self.isCarryingC = False

class Environment(Model):
    def __init__(self, M: int, N: int, num_agents: int = 5, num_goals: int = 1, obstacle_portion: float = 0.3, mode_start_pos='Random'):
        super().__init__()
        self.num_agents = num_agents
        self.num_goals = num_goals
        self.grid = SingleGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)
        self.step_count = 0
        self.num_goals_slider = num_goals  
        self.mode_start_pos = mode_start_pos
        self.special_cell = []

        self.central_system = TaskManager(0, self)
        self.schedule.add(self.central_system)

        self.desc = [
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
        'WFDFDFDFXFFFFFFFFXFFFFFFFFFFFYFXFYFZFW',
        'WFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFRFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFRFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFRFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WHFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFHW',
        'WFFZFFFFFFFFFFFFFYFFFFFFFFFFFFFRFFFFFW',
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
    ]

        self._manual_placement(self.desc)

        self.datacollector = DataCollector(
            model_reporters={
                "TotalMovements": lambda m: np.sum([a.movements for a in m.schedule.agents if isinstance(a, Bot)]),
            }
        )

    def _manual_placement(self, desc):
        goalPos = (0, 0)
        box_position = []

        N = self.grid.height  # Rows
        M = self.grid.width  # Columns

        for pos in self.grid.coord_iter():
            _, (x, y) = pos
            if y < len(desc) and x < len(desc[y]):
                if desc[y][x] == 'A':  
                    shelf = ShelfA(self.next_id(), self, capacity=3)
                    self.grid.place_agent(shelf, (x, y))
                if desc[y][x] == 'B':  
                    shelf = ShelfB(self.next_id(), self, capacity=3)
                    self.grid.place_agent(shelf, (x, y))
                if desc[y][x] == 'C':  
                    shelf = ShelfC(self.next_id(), self, capacity=3)
                    self.grid.place_agent(shelf, (x, y))
                elif desc[y][x] == 'W':  
                    wall = Wall(self.next_id(), self)
                    self.grid.place_agent(wall, (x, y))
                elif desc[y][x] == 'X':  
                    packagea = (x, y)
                    packageA = PackageA(self.next_id(), self)
                    self.special_cell.append(packagea)
                    box_position.append((x, y))
                    self.grid.place_agent(packageA, (x, y))
                elif desc[y][x] == 'Y':  
                    packageb = (x, y)
                    packageB = PackageB(self.next_id(), self)
                    self.special_cell.append(packageB)
                    box_position.append((x, y))
                    self.grid.place_agent(packageB, (x, y))
                elif desc[y][x] == 'Z':  
                    packagec = (x, y)
                    packageC = PackageC(self.next_id(), self)
                    self.special_cell.append(packageC)
                    box_position.append((x, y))
                    self.grid.place_agent(packageC, (x, y))
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
        self.running = any([isinstance(a, Bot) for a in self.schedule.agents])

