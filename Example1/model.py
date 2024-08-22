import numpy as np
from mesa.model import Model
from mesa.agent import Agent
from mesa.space import SingleGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

class Shelf(Agent):
    def __init__(self, unique_id, model, capacity=3):
        super().__init__(unique_id, model)
        self.capacity = capacity

    def has_capacity(self):
        return self.capacity > 0

    def decrement_capacity(self):
        if self.capacity > 0:
            self.capacity -= 1

class EPackage(Agent):
    def _init_(self, unique_id, model):
        super()._init_(unique_id, model)

class SPackage(Agent):
    def _init_(self, unique_id, model):
        super()._init_(unique_id, model)

class Charger(Agent):
    def _init_(self, unique_id, model):
        super()._init_(unique_id, model)

class Wall(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Goal(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Bot(Agent):
    def __init__(self, unique_id, model, goal=None, target_shelf=None):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.luck = np.random.uniform(0.2, 1.0)
        self.movements = 0
        self.battery = 100
        self.path = []
        self.goal = goal
        self.carrying = False
        self.target_shelf = target_shelf 

    def step(self):
        if self.battery > 0:
            if self.carrying:
                if not self.target_shelf:
                    self.target_shelf = self.select_random_shelf()
                    if self.target_shelf:
                        print(f"Bot {self.unique_id} selected shelf at {self.target_shelf.pos}")
                    else:
                        print(f"Bot {self.unique_id} could not find a valid shelf.")
                        return

                if self.target_shelf:
                    self.path = self.a_star(self.pos, self.target_shelf.pos)
                    if self.path:
                        self.next_pos = self.path.pop(0)
                        self.movements += 1
                        self.battery -= 1
                        self.model.grid.move_agent(self, self.next_pos)
                        print(f"Bot {self.unique_id} moving to shelf at {self.next_pos}")
                        if self.next_pos == self.target_shelf.pos:
                            if self.target_shelf.has_capacity():
                                self.target_shelf.decrement_capacity()
                                self.carrying = False 
                                self.goal = None
                                self.target_shelf = None  
                                print(f"Bot {self.unique_id} has placed the goal on a shelf.")
                            else:
                                print(f"Shelf at {self.target_shelf.pos} is full.")
            else:
                if not self.goal:
                    self.goal = self.find_closest_goal()
                    if self.goal:
                        print(f"Bot {self.unique_id} found goal at {self.goal.pos}")
                if self.goal:
                    self.path = self.a_star(self.pos, self.goal.pos)
                    if self.path:
                        self.next_pos = self.path.pop(0)
                        print(f"Bot {self.unique_id} moving towards goal at {self.goal.pos}")

                        if self.next_pos == self.goal.pos:
                            if not self.model.grid.is_cell_empty(self.next_pos):
                                self.model.grid.remove_agent(self.goal)
                                self.carrying = True
                                self.target_shelf = self.select_random_shelf() 
                                if self.target_shelf:
                                    print(f"Bot {self.unique_id} has picked up the goal at {self.goal.pos}")
                                else:
                                    print(f"Bot {self.unique_id} could not find a shelf to place the goal.")

                        # Move to the next position after removing the goal
                        self.movements += 1
                        self.battery -= 1
                        self.model.grid.move_agent(self, self.next_pos)
                        print(f"Bot {self.unique_id} moved to {self.next_pos}")



    def find_closest_goal(self):
        goals = [a for a in self.model.schedule.agents if isinstance(a, Goal)]
        if not goals:
            return None
        closest_goal = min(goals, key=lambda g: self.manhattan_heuristic(self.pos, g.pos))
        return closest_goal

    def select_random_shelf(self):
        shelves = [a for a in self.model.schedule.agents if isinstance(a, Shelf) and a.has_capacity()]
        if not shelves:
            print("No available shelves found.")
        else:
            print(f"Available shelves: {[shelf.pos for shelf in shelves]}")
        return self.random.choice(shelves) if shelves else None




    def manhattan_heuristic(self, pos, goal):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def a_star(self, start, goal):
        open_list = []
        closed_list = set()
        open_list.append((0 + self.manhattan_heuristic(start, goal), 0, start, None))

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
                if not self.model.grid.is_cell_empty(neighbor) and neighbor != goal:
                    continue 

                g_new = g + 1
                h_new = self.manhattan_heuristic(neighbor, goal)
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


class Environment(Model):
    def __init__(self, M: int, N: int, num_agents: int = 5, num_goals: int = 1, obstacle_portion: float = 0.3, mode_start_pos='Random'):
        super().__init__()
        self.num_agents = num_agents
        self.num_goals = num_goals
        self.grid = SingleGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)
        self.num_goals_slider = num_goals  
        self.mode_start_pos = mode_start_pos

        self.desc = [
            'WWWWWWWWWWWWWWWWWWW',
            'WDDDFFFFFFFFFFFEEEW',
            'WFFFRFFFFFFFFFFFFFW',
            'WFFFFFFFFFFFFFFFFFW',
            'WFFSSFFFSSFFFSSFFFW',
            'WFFSSFFFSSFFFSSFFFW',
            'WFFFFFFFFFFFFFFFFFW',
            'WCFFFFFFFFFFFFFFFFW',
            'WFFFFFFFFFFFFGFFFFW',
            'WCFFFFFFDDDFFFFFFFW',
            'WWWWWWWWWWWWWWWWWWW',
        ]
        
        self._manual_placement(self.desc)
        #self._automatic_placement(M, N, num_agents, obstacle_portion, mode_start_pos)


        self.datacollector = DataCollector(
            model_reporters={
                "MinBattery": lambda m: np.min([a.battery for a in m.schedule.agents if isinstance(a, Bot)]).astype(float),
                "MaxBattery": lambda m: np.max([a.battery for a in m.schedule.agents if isinstance(a, Bot)]).astype(float),
                "MeanBattery": lambda m: np.mean([a.battery for a in m.schedule.agents if isinstance(a, Bot)]).astype(float),
            }
        )

    def _automatic_placement(self, M, N, num_agents, obstacle_portion, mode_start_pos):

        available_positions = [pos for _, pos in self.grid.coord_iter()]

        # Posicionamiento de obstáculos
        num_obstacles = int(M * N * obstacle_portion)
        obstacle_positions = self.random.sample(available_positions, k=num_obstacles)

        for id, pos in enumerate(obstacle_positions):
            shelf = Shelf(int(f"{num_obstacles}0{id}") + 1, self)
            self.grid.place_agent(shelf, pos)
            available_positions.remove(pos)

        # Posicionamiento de agentes robot
        if mode_start_pos == 'Random':
            bts_pos_init = self.random.sample(available_positions, k=num_agents)
        else:  # 'Fixed'
            bts_pos_init = [(1, 1)] * num_agents

        for id in range(num_agents):
            bot = Bot(id, self)
            self.grid.place_agent(bot, bts_pos_init[id])
            self.schedule.add(bot)


    def _manual_placement(self, desc):
        goals = []
        bots = [] 
        available_positions = [pos for _, pos in self.grid.coord_iter()]

        for i, row in enumerate(desc):
            for j, cell in enumerate(row):
                pos = (j, i)
                if 0 <= pos[0] < self.grid.width and 0 <= pos[1] < self.grid.height:
                    if cell == 'S':  # Obstacle
                        shelf = Shelf(self.next_id(), self, capacity=3)
                        self.grid.place_agent(shelf, pos)
                        available_positions.remove(pos)
                    elif cell == 'W':  # Wall
                        wall = Wall(self.next_id(), self)
                        self.grid.place_agent(wall, pos)
                        available_positions.remove(pos)
                    elif cell == 'G':  # Goal
                        goal = Goal(self.next_id(), self)
                        self.grid.place_agent(goal, pos)
                        goals.append(goal)
                        available_positions.remove(pos)
                    elif cell == 'C':  # charger
                        charger = Charger(self.next_id(), self)
                        self.grid.place_agent(charger, pos)
                        available_positions.remove(pos)
                    elif cell == 'E':  # entrada package
                        entrada = EPackage(self.next_id(), self)
                        self.grid.place_agent(entrada, pos)
                        available_positions.remove(pos)
                    elif cell == 'D':  # salida package
                        salida = SPackage(self.next_id(), self)
                        self.grid.place_agent(salida, pos)
                        available_positions.remove(pos)
                    elif cell == 'R':  
                        bots.append(pos)
                        if pos in available_positions:
                            available_positions.remove(pos)

        for pos in bots:
            closest_goal = min(goals, key=lambda g: abs(g.pos[0] - pos[0]) + abs(g.pos[1] - pos[1]))
            bot = Bot(self.next_id(), self, closest_goal)
            self.grid.place_agent(bot, pos)
            self.schedule.add(bot)
            if pos in available_positions:
                available_positions.remove(pos)



    def add_goals(self, num_new_goals):
        available_positions = [pos for _, pos in self.grid.coord_iter() if self.grid.get_agent_count(pos) == 0]
        goals = [a for a in self.schedule.agents if isinstance(a, Goal)]
        
        if len(available_positions) < num_new_goals:
            print("Not enough space to place all new goals.")
            return

        for _ in range(num_new_goals):
            pos = self.random.choice(available_positions)
            goal = Goal(self.next_id(), self)
            self.grid.place_agent(goal, pos)
            goals.append(goal)
            available_positions.remove(pos)
        
        self.num_goals_slider += num_new_goals

    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()
        self.running = any([a.battery > 0 for a in self.schedule.agents if isinstance(a, Bot)])
