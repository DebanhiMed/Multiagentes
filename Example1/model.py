import numpy as np
from mesa.model import Model
from mesa.agent import Agent
from mesa.space import SingleGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

class Shelf(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Goal(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Bot(Agent):
    def __init__(self, unique_id, model, goal=None):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.luck = np.random.uniform(0.2, 1.0)
        self.movements = 0
        self.battery = 100
        self.path = []
        self.goal = goal

    def step(self):
        if self.battery > 0 and self.path:
            self.next_pos = self.path.pop(0)
            if self.next_pos == self.goal.pos:
                # Remove the goal before moving the robot into the goal's position
                if self.goal in self.model.schedule._agents:
                    self.model.grid.remove_agent(self.goal)
                    self.model.schedule.remove(self.goal)
                self.goal = None  # Robot no longer has a goal
            self.movements += 1
            self.battery -= 1
            self.model.grid.move_agent(self, self.next_pos)

        elif self.battery > 0 and self.goal:
            self.path = self.a_star(self.pos, self.goal.pos)
            if self.path:
                self.next_pos = self.path.pop(0)
                if self.next_pos == self.goal.pos:
                    # Remove the goal before moving the robot into the goal's position
                    if self.goal in self.model.schedule._agents:
                        self.model.grid.remove_agent(self.goal)
                        self.model.schedule.remove(self.goal)
                    self.goal = None  # Robot no longer has a goal
                self.movements += 1
                self.battery -= 1
                self.model.grid.move_agent(self, self.next_pos)


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
                    continue  # Skip cells that are occupied

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
        self.num_goals_slider = num_goals  # To control number of goals with slider
        self.mode_start_pos = mode_start_pos

        self.desc = [
            'FFFFFFFFFFFFFFFFFF',
            'FFFFFFFFFFFFFFFFFF',
            'FFFFFFFFFFFFFFFFFF',
            'FFFFFFFFFBBBFFFFFF',
            'FFFFFFFFFBBBFFFFFF',
            'FFFFFFFFFBBBBBBFFF',
            'FFFFFFFFFFFFFFFFFF',
            'FFFFFFFFFFFFFFFFFF',
            'FFFFFFFFFFFFFFFFFF',
            'FFFFFFFFFFFFFFFFFF'
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
        available_positions = [pos for _, pos in self.grid.coord_iter()]

        for i, row in enumerate(desc):
            for j, cell in enumerate(row):
                pos = (j, i)
                if 0 <= pos[0] < self.grid.width and 0 <= pos[1] < self.grid.height:
                    if cell == 'B':  # Obstacle
                        shelf = Shelf(self.next_id(), self)
                        self.grid.place_agent(shelf, pos)
                        if pos in available_positions:
                            available_positions.remove(pos)
                    elif cell == 'G':  # Goal
                        goal = Goal(self.next_id(), self)
                        self.grid.place_agent(goal, pos)
                        goals.append(goal)
                        if pos in available_positions:
                            available_positions.remove(pos)

        # Place bots
        if goals:  # Check if there are any goals
            for _ in range(self.num_agents):
                if not available_positions:
                    break  # Stop if no positions left

                pos = self.random.choice(available_positions)
                closest_goal = min(goals, key=lambda g: abs(g.pos[0] - pos[0]) + abs(g.pos[1] - pos[1]))
                bot = Bot(self.next_id(), self, closest_goal)
                self.grid.place_agent(bot, pos)
                self.schedule.add(bot)
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
