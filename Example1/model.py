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
        'WFDFDFDFFFFFFFFFFFFFFFFFFFFFFYFXFYFZFW',
        'WFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WHFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
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


"""
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


class Charger(Agent):
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
        self.battery = 500
        self.path = []
        self.carry = False
        self.tasks = []
        self.goal = None
        #self.charging = False
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
        
        # Check if the battery is low and the bot needs to charge, but only if not carrying a goal
        if self.battery < 460 and not self.carry and not self.charging:
            charger_pos = self.find_charger()
            if charger_pos:
                self.goal = charger_pos
                self.path = self.a_star(self.pos, self.goal)
                self.charging = True

        if self.charging:
            if self.is_adjacent_to_charger(self.pos):  # At the charger or adjacent to it
                self.battery += 100  # Recharge
                if self.battery >= 500:  # Fully charged
                    self.battery = 500
                    self.charging = False

                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)
                    else:
                        self.find_new_task()
            else:  # Move towards the charger
                if self.path:
                    self.next_pos = self.path.pop(0)
                    if self.detect_collision(self.next_pos):
                        self.path = self.find_alternative_path(self.pos, self.goal)
                        if self.path:
                            self.next_pos = self.path.pop(0)
                        else:
                            return  # No alternative path found, wait in place
                    self.model.grid.move_agent(self, self.next_pos)
                    self.movements += 1
                    self.battery = max(0, self.battery - 1)  # Deplete battery slightly while moving
            return
    
        # Normal operations (if not charging)
        if self.battery > 0 and self.path and self.goal:
            self.next_pos = self.path.pop(0)

            if self.detect_collision(self.next_pos):
                self.path = self.find_alternative_path(self.pos, self.goal)
                if not self.path:
                    return  # No alternative path found, wait in place
                self.next_pos = self.path.pop(0)

            self.movements += 1
            self.battery = max(0, self.battery - 1)
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
                    # Interact with the SPackage without removing it
                    self.carry = False
                    self.reset_carrying_flags()
                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)
                        if self.path:
                            self.next_pos = self.path.pop(0)
                            self.model.grid.move_agent(self, self.next_pos)
                            self.movements += 1
                            self.battery = max(0, self.battery - 1)
                else:
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.model.grid.move_agent(self, self.next_pos)

        elif self.battery > 0 and self.goal is None and not self.path:
            if self.tasks:
                self.goal = self.tasks.pop()[0]
                self.path = self.a_star(self.pos, self.goal)
                if self.path:
                    self.next_pos = self.path.pop(0)
                    self.movements += 1
                    self.battery = max(0, self.battery - 1)
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.find_new_task()

    def find_new_task(self):
        # Look for a new task after charging or completing a task
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
        'WFDFDFDFFFFFFFFFFFFFFFFFFFFFFYFXFYFZFW',
        'WFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WHFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFHW',
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

                elif desc[y][x] == 'H':  
                    charger = Charger(self.next_id(), self)
                    self.grid.place_agent(charger, (x, y))
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
        self.running = any([a.battery > 0 for a in self.schedule.agents if isinstance(a, Bot)])


////LND;BDJNLDCJBC

"""
"""
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

    def step(self):
        # Asignar tareas a los robots libres
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot):
                if not bot.tasks:
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

class Charger(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Wall(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Bot(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.path = []
        self.carry = False
        self.tasks = []
        self.goal = None

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

    def avoid_collision(self):
        for neighbor in self.model.grid.iter_neighborhood(self.pos, moore=False, include_center=False):
            if self.model.grid.is_cell_empty(neighbor):
                return neighbor
        return self.pos

    def step(self):
        if self.path and self.goal:
            self.next_pos = self.path.pop(0)

            if self.detect_collision(self.next_pos):
                self.path = self.a_star(self.pos, self.goal)
                if not self.path:
                    return  # No alternative path found, wait in place
                self.next_pos = self.path.pop(0)

            self.model.grid.move_agent(self, self.next_pos)

            if not self.carry:
                box_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, (PackageA, PackageB, PackageC))]
                if box_in_next_pos:
                    goal = box_in_next_pos[0]
                    self.model.grid.remove_agent(goal)
                    self.carry = True
                    self.goal = self.find_exit()
                    self.path = self.a_star(self.pos, self.goal)

            if self.carry:
                exit_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, SPackage)]
                if exit_in_next_pos:
                    self.carry = False
                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)
                else:
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.model.grid.move_agent(self, self.next_pos)

        elif not self.tasks:
            if self.tasks:
                self.goal = self.tasks.pop()[0]
                self.path = self.a_star(self.pos, self.goal)
                self.model.grid.move_agent(self, self.next_pos)

class Environment(Model):
    def __init__(self, M: int, N: int, num_agents: int = 5, num_goals: int = 1, obstacle_portion: float = 0.3, mode_start_pos='Random'):
        super().__init__()
        self.num_agents = num_agents
        self.num_goals = num_goals
        self.grid = SingleGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)
        self.step_count = 0

        self.central_system = TaskManager(self.next_id(), self)
        self.schedule.add(self.central_system)

        self.desc = [
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
        'WFDFDFDFFFFFFFFFFFFFFFFFFFFFFYFXFYFZFW',
        'WFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WHFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
    ]
                                                  
        # Llamada al método _manual_placement para ubicar los agentes según el diseño de la cuadrícula
        self._manual_placement(self.desc)

        # Configuración del DataCollector, eliminando cualquier referencia a la batería
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
                elif desc[y][x] == 'B':  
                    shelf = ShelfB(self.next_id(), self, capacity=3)
                    self.grid.place_agent(shelf, (x, y))
                elif desc[y][x] == 'C':  
                    shelf = ShelfC(self.next_id(), self, capacity=3)
                    self.grid.place_agent(shelf, (x, y))
                elif desc[y][x] == 'W':  
                    wall = Wall(self.next_id(), self)
                    self.grid.place_agent(wall, (x, y))
                elif desc[y][x] == 'H':  
                    charger = Charger(self.next_id(), self)
                    self.grid.place_agent(charger, (x, y))
                elif desc[y][x] == 'X':  
                    packageA = PackageA(self.next_id(), self)
                    self.grid.place_agent(packageA, (x, y))
                    self.central_system.add_task((x, y))
                elif desc[y][x] == 'Y':  
                    packageB = PackageB(self.next_id(), self)
                    self.grid.place_agent(packageB, (x, y))
                    self.central_system.add_task((x, y))
                elif desc[y][x] == 'Z':  
                    packageC = PackageC(self.next_id(), self)
                    self.grid.place_agent(packageC, (x, y))
                    self.central_system.add_task((x, y))
                elif desc[y][x] == 'D':  
                    salida = SPackage(self.next_id(), self)
                    self.grid.place_agent(salida, (x, y))
                elif desc[y][x] == 'R':  
                    bot = Bot(self.next_id(), self)
                    self.grid.place_agent(bot, (x, y))
                    self.schedule.add(bot)

    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()
        self.running = any([isinstance(a, Bot) for a in self.schedule.agents])







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
        self.tasks_A = []  # Lista de tareas pendientes para PackageA
        self.tasks_B = []  # Lista de tareas pendientes para PackageB
        self.tasks_C = []  # Lista de tareas pendientes para PackageC
        self.robot_statuses = {}

    def step(self):
        # Asignar tareas a los robots libres
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot) and not bot.tasks:
                if bot.preferred_package == "A" and self.tasks_A:
                    task = self.tasks_A.pop(0)
                    bot.getTasks(task)
                elif bot.preferred_package == "B" and self.tasks_B:
                    task = self.tasks_B.pop(0)
                    bot.getTasks(task)
                elif bot.preferred_package == "C" and self.tasks_C:
                    task = self.tasks_C.pop(0)
                    bot.getTasks(task)

    def add_task(self, task, package_type):
        if package_type == "A":
            self.tasks_A.append(task)
        elif package_type == "B":
            self.tasks_B.append(task)
        elif package_type == "C":
            self.tasks_C.append(task)

    def report_completion(self, robot_id):
        self.robot_statuses[robot_id] = "Free"

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


class Charger(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Wall(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Bot(Agent):
    def __init__(self, unique_id, model, preferred_package):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.luck = np.random.uniform(0.2, 1.0)
        self.movements = 0
        self.battery = 500
        self.path = []
        self.carry = False
        self.tasks = []
        self.goal = None
        self.charging = False
        self.preferred_package = preferred_package  # "A", "B", or "C"

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

    def find_charger(self):
        for pos in self.model.grid.coord_iter():
            _, (x, y) = pos
            contents = self.model.grid.get_cell_list_contents([x, y])
            for obj in contents:
                if isinstance(obj, Charger):
                    return (x, y)
        return None

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

    

    def avoid_collision(self):
        for neighbor in self.model.grid.iter_neighborhood(self.pos, moore=False, include_center=False):
            if self.model.grid.is_cell_empty(neighbor):
                return neighbor
        return self.pos
    
    def is_adjacent_to_charger(self, pos):
        x, y = pos
        neighbors = [(x + dx, y + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1] if dx != 0 or dy != 0]
        for neighbor in neighbors:
            if any(isinstance(agent, Charger) for agent in self.model.grid.get_cell_list_contents(neighbor)):
                return neighbor
        return None
    
    def find_alternative_path(self, start, goal):
        avoid_positions = [agent.next_pos for agent in self.model.schedule.agents if isinstance(agent, Bot) and agent.next_pos is not None]
        return self.a_star(start, goal, avoid_positions)
    
    def step(self):
        # Check if the battery is low and the bot needs to charge
        if self.battery < 200 and not self.carry and not self.charging:
            charger_pos = self.find_charger()
            if charger_pos:
                self.goal = charger_pos
                self.path = self.a_star(self.pos, self.goal)
                self.charging = True

        if self.charging:
            if self.model.grid.get_cell_list_contents([self.pos]) == [self.goal]:  # At the charger
                self.battery += 100  # Recharge
                if self.battery >= 500:  # Fully charged
                    self.battery = 500
                    self.charging = False
                    self.goal = None
                    self.path = []
            else:  # Move towards the charger
                if self.path:
                    self.next_pos = self.path.pop(0)
                    if self.detect_collision(self.next_pos):
                        self.next_pos = self.avoid_collision()
                    self.model.grid.move_agent(self, self.next_pos)
                    self.battery -= 1  # Deplete battery while moving
            return

        # Normal operations (if not charging)
        if self.battery > 0 and self.path and self.goal:
            self.next_pos = self.path.pop(0)

            if self.detect_collision(self.next_pos):
                self.next_pos = self.avoid_collision()

            if self.model.grid.is_cell_empty(self.next_pos):
                self.model.grid.move_agent(self, self.next_pos)
                self.battery -= 1
                self.movements += 1

            if not self.carry:
                package_type = {"A": PackageA, "B": PackageB, "C": PackageC}[self.preferred_package]
                box_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, package_type)]
                if box_in_next_pos:
                    goal = box_in_next_pos[0]
                    self.model.grid.remove_agent(goal)
                    self.carry = True
                    self.goal = self.find_exit()
                    self.path = self.a_star(self.pos, self.goal)

            if self.carry:
                exit_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, SPackage)]
                if exit_in_next_pos:
                    self.carry = False
                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)

        elif self.battery > 0 and not self.tasks:
            if self.tasks:
                self.goal = self.tasks.pop()[0]
                self.path = self.a_star(self.pos, self.goal)


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

        self.central_system = TaskManager(self.next_id(), self)
        self.schedule.add(self.central_system)

        self.desc = [
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
        'WFDFDFDFFFFFFFFFFFFFFFFFFFFFFYFXFYFZFW',
        'WFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WHFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFHW',
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

                elif desc[y][x] == 'H':  
                    charger = Charger(self.next_id(), self)
                    self.grid.place_agent(charger, (x, y))
                elif desc[y][x] == 'X':  
                    goal = (x, y)
                    goall = PackageA(self.next_id(), self)
                    self.special_cell.append(goal)
                    box_position.append((x, y))
                    self.grid.place_agent(goall, (x, y))
                elif desc[y][x] == 'Y':  
                    entrada = PackageB(self.next_id(), self)
                    self.grid.place_agent(entrada, (x, y))
                elif desc[y][x] == 'Z':  
                    entrada = PackageC(self.next_id(), self)
                    self.grid.place_agent(entrada, (x, y))
                elif desc[y][x] == 'D':  
                    salida = SPackage(self.next_id(), self)
                    self.grid.place_agent(salida, (x, y))
                elif desc[y][x] == 'R':  
                    bot = Bot(self.next_id(), self, preferred_package="A")  # Cambia "A" a "B" o "C" según el bot
                    self.grid.place_agent(bot, (x, y))
                    self.schedule.add(bot)

        for box_pos in box_position:
            self.central_system.add_task((box_pos, goalPos), "A")  # Cambia "A" a "B" o "C" según el tipo de paquete


    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()
        self.running = any([a.battery > 0 for a in self.schedule.agents if isinstance(a, Bot)])
"""
"""
    TO DO: 
     - Hacer que los robots se rodeen si van a colisionar (no intercambiar posiciones)
     - Hacer que el EPackage entregue paquetes
     - Agregar ids a los paquetes
     - Ordenar paquetes en sus respectivos estantes
     - Hacer que los robots se carguen cuando tengan poca bateria
     - Hacer que los cargadores funcionen
     - Hacer que la mitad de los robots sean de entrega y otros de acomodado



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
        self.charger_occupancy = {}  # Tracks which chargers are occupied

    def step(self):
        # Asignar tareas a los robots libres
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot):
                if not bot.charging and self.tasks:
                    task = self.tasks.pop(0)
                    bot.getTasks(task)
    
    def assign_charger(self, bot):
        # Assign the closest unoccupied charger to the bot
        closest_charger = None
        min_distance = float('inf')
        for charger in self.model.grid.get_neighbors(bot.pos, include_center=True):
            if isinstance(charger, Charger) and charger.unique_id not in self.charger_occupancy:
                distance = bot.euclidean_heuristic(bot.pos, charger.pos)
                if distance < min_distance:
                    closest_charger = charger
                    min_distance = distance

        if closest_charger:
            self.charger_occupancy[closest_charger.unique_id] = bot.unique_id
            bot.assigned_charger = closest_charger.pos
            bot.goal = closest_charger.pos
            bot.path = bot.a_star(bot.pos, bot.goal)
            bot.charging = True

    def release_charger(self, charger_id):
        # Free up the charger when the bot is done charging
        if charger_id in self.charger_occupancy:
            del self.charger_occupancy[charger_id]
    

    def report_completion(self, robot_id):
        self.robot_statuses[robot_id] = "Free"

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


class Charger(Agent):
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
        self.battery = 500
        self.path = []
        self.carry = False
        self.tasks = []
        self.goal = None
        #self.charging = False
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
    
    def find_charger(self):
        for pos in self.model.grid.coord_iter():
            _, (x, y) = pos
            contents = self.model.grid.get_cell_list_contents([x, y])
            for obj in contents:
                if isinstance(obj, Charger):
                    return (x, y)
        return None
    
    def is_adjacent_to_charger(self, pos):
        x, y = self.pos
        neighbors = [(x + dx, y + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1] if dx != 0 or dy != 0]
        for neighbor in neighbors:
            if neighbor == self.assigned_charger:
                return True
        return False

    //checar que es la diferencia entre estas dos funciones??

    def is_adjacent_to_charger(self, pos):
        x, y = pos
        neighbors = [(x + dx, y + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1] if dx != 0 or dy != 0]
        for neighbor in neighbors:
            if any(isinstance(agent, Charger) for agent in self.model.grid.get_cell_list_contents(neighbor)):
                return neighbor
        return None
    

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
        
        # Check if the battery is low and the bot needs to charge, but only if not carrying a goal
        if self.battery < 460 and not self.carry and not self.charging:
            charger_pos = self.find_charger()
            if charger_pos:
                self.goal = charger_pos
                self.path = self.a_star(self.pos, self.goal)
                self.charging = True

        if self.charging:
            if self.is_adjacent_to_charger(self.pos):  # At the charger or adjacent to it
                self.battery += 100  # Recharge
                if self.battery >= 500:  # Fully charged
                    self.battery = 500
                    self.charging = False

                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)
                    else:
                        self.find_new_task()
            else:  # Move towards the charger
                if self.path:
                    self.next_pos = self.path.pop(0)
                    if self.detect_collision(self.next_pos):
                        self.path = self.find_alternative_path(self.pos, self.goal)
                        if self.path:
                            self.next_pos = self.path.pop(0)
                        else:
                            return  # No alternative path found, wait in place
                    self.model.grid.move_agent(self, self.next_pos)
                    self.movements += 1
                    self.battery = max(0, self.battery - 1)  # Deplete battery slightly while moving
            return
    
        # Normal operations (if not charging)
        if self.battery > 0 and self.path and self.goal:
            self.next_pos = self.path.pop(0)

            if self.detect_collision(self.next_pos):
                self.path = self.find_alternative_path(self.pos, self.goal)
                if not self.path:
                    return  # No alternative path found, wait in place
                self.next_pos = self.path.pop(0)

            self.movements += 1
            self.battery = max(0, self.battery - 1)
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
                    # Interact with the SPackage without removing it
                    self.carry = False
                    self.reset_carrying_flags()
                    if self.tasks:
                        self.goal = self.tasks.pop()[0]
                        self.path = self.a_star(self.pos, self.goal)
                        if self.path:
                            self.next_pos = self.path.pop(0)
                            self.model.grid.move_agent(self, self.next_pos)
                            self.movements += 1
                            self.battery = max(0, self.battery - 1)
                else:
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.model.grid.move_agent(self, self.next_pos)

        elif self.battery > 0 and self.goal is None and not self.path:
            if self.tasks:
                self.goal = self.tasks.pop()[0]
                self.path = self.a_star(self.pos, self.goal)
                if self.path:
                    self.next_pos = self.path.pop(0)
                    self.movements += 1
                    self.battery = max(0, self.battery - 1)
                    self.model.grid.move_agent(self, self.next_pos)
            else:
                self.find_new_task()

    def find_new_task(self):
        # Look for a new task after charging or completing a task
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
        'WFDFDFDFFFFFFFFFFFFFFFFFFFFFFYFXFYFZFW',
        'WFFFRFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WHFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFHW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WHFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFHW',
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

                elif desc[y][x] == 'H':  
                    charger = Charger(self.next_id(), self)
                    self.grid.place_agent(charger, (x, y))
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
        self.running = any([a.battery > 0 for a in self.schedule.agents if isinstance(a, Bot)])
"""
