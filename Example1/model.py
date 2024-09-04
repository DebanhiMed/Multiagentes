import numpy as np
import random
import json
from mesa import Model, Agent
from mesa.space import SingleGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

class ShelfA(Agent):
    def __init__(self, unique_id, model, capacity=3):
        super().__init__(unique_id, model)
        self.capacity = capacity
        self.is_full = False

    def has_capacity(self):
        return self.capacity > 0

    def decrement_capacity(self):
        if self.capacity > 0:
            self.capacity -= 1
            if self.capacity == 0:
                self.is_full = True  # Mark the shelf as full when capacity reaches 0

    def increment_capacity(self):
        if self.capacity < 3:
            self.capacity += 1
            if self.is_full and self.capacity > 0:
                self.is_full = False  # Mark the shelf as not full when capacity is increased

class ShelfB(Agent):
    def __init__(self, unique_id, model, capacity=3):
        super().__init__(unique_id, model)
        self.capacity = capacity
        self.is_full = False

    def has_capacity(self):
        return self.capacity > 0

    def decrement_capacity(self):
        if self.capacity > 0:
            self.capacity -= 1
            if self.capacity == 0:
                self.is_full = True  # Mark the shelf as full when capacity reaches 0

    def increment_capacity(self):
        if self.capacity < 3:
            self.capacity += 1
            if self.is_full and self.capacity > 0:
                self.is_full = False  # Mark the shelf as not full when capacity is increased

class ShelfC(Agent):
    def __init__(self, unique_id, model, capacity=3):
        super().__init__(unique_id, model)
        self.capacity = capacity
        self.is_full = False

    def has_capacity(self):
        return self.capacity > 0

    def decrement_capacity(self):
        if self.capacity > 0:
            self.capacity -= 1
            if self.capacity == 0:
                self.is_full = True  # Mark the shelf as full when capacity reaches 0

    def increment_capacity(self):
        if self.capacity < 3:
            self.capacity += 1
            if self.is_full and self.capacity > 0:
                self.is_full = False  # Mark the shelf as not full when capacity is increased


class TaskManager(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.tasks = []

    def step(self):
        self.update_tasks()

        # Asignar tareas a los robots libres en cada step
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot) and not bot.tasks and not bot.carry and not bot.returning_to_start:
                if self.tasks:
                    task = self.tasks.pop(0)
                    bot.getTasks(task)
                    bot.goal = task[0]
                    bot.path = bot.a_star(bot.pos, bot.goal)
                    print(f"Robot {bot.unique_id} recibió un nuevo task hacia {bot.goal}")


    def add_task(self, task):
        self.tasks.append(task)

    def update_tasks(self):
        for pos in self.model.grid.coord_iter():
            _, agents = pos
            for agent in agents:
                if isinstance(agent, (PackageA, PackageB, PackageC)) and not any(task[0] == agent.pos for task in self.tasks):
                    self.add_task((agent.pos, None))


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

class EPackage(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.spawn_directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        self.current_direction = 0  # Start facing right
        self.timer = random.randint(50, 100)  # Temporizador para controlar el spawn de paquetes

    def step(self):
        direction = self.spawn_directions[self.current_direction]
        spawn_position = (self.pos[0] + direction[0], self.pos[1] + direction[1])

        if not self.model.grid.is_cell_empty(spawn_position):
            pass
        else:
            self.timer -= 1

        if self.timer <= 0 and self.model.grid.is_cell_empty(spawn_position):
            package_type = random.choice([PackageA, PackageB, PackageC])
            new_package = package_type(self.model.next_id(), self.model)
            self.model.grid.place_agent(new_package, spawn_position)
            self.model.central_system.add_task((spawn_position, None))
            self.timer = random.randint(1, 1)

class Wall(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Bot(Agent):
    def __init__(self, unique_id, model, initial_position):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.movements = 0
        self.battery = 100  
        self.path = []
        self.carry = False
        self.tasks = []
        self.goal = None
        self.initial_position = initial_position
        self.returning_to_start = False
        self.isCarryingA = False
        self.isCarryingB = False
        self.isCarryingC = False
        self.history = {
            "spawnPosition": {"x": initial_position[0], "y": initial_position[1]},
            "path": [],
            "deliveryPoints": [],
            "pickupPoints": []
        }

    def getTasks(self, task):
        self.tasks.append(task)

    def euclidean_heuristic(self, pos, goal):
        return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
        
    def find_alternative_path(self, start, goal):
        avoid_positions = [agent.next_pos for agent in self.model.schedule.agents if isinstance(agent, Bot) and agent.next_pos is not None]
        alternative_path = self.a_star(start, goal, avoid_positions)
        
        if not alternative_path:
            alternative_path = self.a_star(start, goal)
        
        return alternative_path

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
                        continue
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

    def find_adjacent_empty_cell(self, target_pos):
        neighbors = self.model.grid.get_neighborhood(target_pos, moore=False, include_center=False)
        for neighbor in neighbors:
            if self.model.grid.is_cell_empty(neighbor):
                return neighbor
        return None

    def find_shelf(self, package_type):
        shelf_type = ShelfA if package_type == 1 else ShelfB if package_type == 2 else ShelfC
        shelves = []
        for pos in self.model.grid.coord_iter():
            _, (x, y) = pos
            contents = self.model.grid.get_cell_list_contents([x, y])
            for obj in contents:
                if isinstance(obj, shelf_type) and not obj.is_full and obj.has_capacity():
                    shelves.append((x, y))
        
        if shelves:
            shelves.sort(key=lambda shelf: self.euclidean_heuristic(self.pos, shelf))
            return shelves[0]
        else:
            return None

    def step(self):
        self.history["path"].append({"x": self.pos[0], "y": self.pos[1]})

        # Verificar si el shelf objetivo está lleno en cada paso
        if self.goal:
            shelf_at_goal = self.get_shelf_at_position(self.goal)
            if shelf_at_goal and shelf_at_goal.is_full:
                print(f"Shelf at {self.goal} is full. Redirecting robot {self.unique_id}.")
                self.goal = self.find_shelf(1 if self.isCarryingA else 2 if self.isCarryingB else 3)
                if self.goal:
                    self.path = self.a_star(self.pos, self.goal)

        if self.path and self.goal:
            self.next_pos = self.path.pop(0)
            if self.next_pos is None or self.model.grid.out_of_bounds(self.next_pos):
                print(f"Warning: next_pos is not valid: {self.next_pos}")
                return

            if self.detect_collision(self.next_pos):
                self.path = self.find_alternative_path(self.pos, self.goal)
                if not self.path:
                    return  
                self.next_pos = self.path.pop(0)
                if self.next_pos is None or self.model.grid.out_of_bounds(self.next_pos):
                    print(f"Warning: next_pos after alternative path is not valid: {self.next_pos}")
                    return

            self.movements += 1

            if not self.carry:
                box_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, (PackageA, PackageB, PackageC))]
                if box_in_next_pos:
                    goal = box_in_next_pos[0]
                    self.model.grid.remove_agent(goal)
                    self.carry = True
                    self.update_carrying_flags(goal)

                    self.history["pickupPoints"].append({"x": self.next_pos[0], "y": self.next_pos[1]})

                    if isinstance(goal, PackageA):
                        package_type = 1
                    elif isinstance(goal, PackageB):
                        package_type = 2
                    elif isinstance(goal, PackageC):
                        package_type = 3

                    self.goal = self.find_shelf(package_type)
                    self.path = self.a_star(self.pos, self.goal)

        if self.carry:
            shelf_pos = [(self.pos[0] + dx, self.pos[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
            shelf_in_adjacent_pos = [pos for pos in shelf_pos if any(isinstance(agent, (ShelfA, ShelfB, ShelfC)) and not agent.is_full for agent in self.model.grid.get_cell_list_contents([pos]))]

            if shelf_in_adjacent_pos:
                shelf = next(agent for pos in shelf_in_adjacent_pos for agent in self.model.grid.get_cell_list_contents([pos]) if isinstance(agent, (ShelfA, ShelfB, ShelfC)) and not agent.is_full)

                if isinstance(shelf, ShelfA) and self.isCarryingA and shelf.has_capacity():
                    shelf.decrement_capacity()
                    if not shelf.has_capacity():
                        shelf.is_full = True
                    self.isCarryingA = False
                elif isinstance(shelf, ShelfB) and self.isCarryingB and shelf.has_capacity():
                    shelf.decrement_capacity()
                    if not shelf.has_capacity():
                        shelf.is_full = True
                    self.isCarryingB = False
                elif isinstance(shelf, ShelfC) and self.isCarryingC and shelf.has_capacity():
                    shelf.decrement_capacity()
                    if not shelf.has_capacity():
                        shelf.is_full = True
                    self.isCarryingC = False
                else:
                    return

                self.carry = False
                self.history["deliveryPoints"].append({"x": self.pos[0], "y": self.pos[1]})
                print(f"Robot {self.unique_id} dejó una caja en {self.pos}")

                self.goal = None
                self.path = []
                self.reset_carrying_flags()

                self.model.central_system.step()
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
            elif self.returning_to_start:
                self.goal = self.initial_position
                self.path = self.a_star(self.pos, self.goal)
                if self.path:
                    self.next_pos = self.path.pop(0)
                    self.movements += 1
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                return
        else:
            self.model.grid.move_agent(self, self.next_pos)

    def get_shelf_at_position(self, pos):
        contents = self.model.grid.get_cell_list_contents([pos])
        for obj in contents:
            if isinstance(obj, (ShelfA, ShelfB, ShelfC)):
                return obj
        return None

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

        # Almacenar los estantes llenos
        self.full_shelves = set()

        bot_id = 1

        self.desc = [
        'WWWWWWWWWWWWWWWWWWWWWWWEWWWWEWWWWEWWWW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'DFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'DFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'DFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFRFFRFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
    ]

        self._manual_placement(self.desc, bot_id)

        self.datacollector = DataCollector(
            model_reporters={
                "TotalMovements": lambda m: np.sum([a.movements for a in m.schedule.agents if isinstance(a, Bot)]),
            }
        )

    def _manual_placement(self, desc, bot_id):
        goalPos = (0, 0)
        box_position = []

        N = self.grid.height 
        M = self.grid.width

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
                elif desc[y][x] == 'E':  
                    entrada = EPackage(self.next_id(), self)
                    self.grid.place_agent(entrada, (x, y))
                    self.schedule.add(entrada)
                elif desc[y][x] == 'R':  
                    bot = Bot(bot_id, self, (x, y))
                    bot.initial_position = (x, y)
                    self.grid.place_agent(bot, (x, y))
                    self.schedule.add(bot)
                    bot_id += 1

        for box_pos in box_position:
            self.central_system.add_task((box_pos, goalPos))

    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()

        robots_data = {"robots": []}
        for agent in self.schedule.agents:
            if isinstance(agent, Bot):
                robots_data["robots"].append(agent.history)

        with open("resultados_simulacion.json", "w") as json_file:
            json.dump(robots_data, json_file, indent=4)

        self.running = any([isinstance(a, Bot) for a in self.schedule.agents])
