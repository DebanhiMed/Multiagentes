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
                self.is_full = True  # Mark the shelf as full when capacity is 0
            self.model.central_system.add_shelf_task((self.pos, None))

    def increment_capacity(self):
        if self.capacity < 3:
            self.capacity += 1
            if self.is_full and self.capacity > 0:
                self.is_full = False  # Shelf is no longer full

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
                self.is_full = True  # Marcar como lleno cuando la capacidad sea 0
            # Añadir una tarea para que un RBot venga a recoger del estante
            self.model.central_system.add_shelf_task((self.pos, None))

    def increment_capacity(self):
        if self.capacity < 3:
            self.capacity += 1
            if self.is_full and self.capacity > 0:
                self.is_full = False  # El estante ya no está lleno

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
                    self.is_full = True  # Marcar como lleno cuando la capacidad sea 0
                # Añadir una tarea para que un RBot venga a recoger del estante
                self.model.central_system.add_shelf_task((self.pos, None))

    def increment_capacity(self):
            if self.capacity < 3:
                self.capacity += 1
                if self.is_full and self.capacity > 0:
                    self.is_full = False  # El estante ya no está lleno

class Plant(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Extintor(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Cafe1(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Cafe2(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Inclusion(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class TaskManager(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.tasks = []
        self.shelf_tasks = []
        self.assigned_shelves = {} 
        self.occupied_spackages = set()
        self.step_counter = 0
        self.current_order = None
        self.custom_order = {
            "PackageA": random.randint(1, 3),  # Pedidos de cada paquete
            "PackageB": random.randint(1, 3),
            "PackageC": random.randint(1, 3),
            "fulfilled": {
                "PackageA": 0,
                "PackageB": 0,
                "PackageC": 0
            }
        }

    def step(self):
        self.update_tasks()
        self.update_shelf_tasks()
        self.step_counter += 1
        
        if self.step_counter == 1 or self.step_counter % 500 == 0:
            self.create_new_order()

        # Asignar tareas a los Bot regulares
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot) and not bot.tasks and not bot.carry and not bot.returning_to_start:
                if self.tasks:
                    task = self.tasks.pop(0)
                    if isinstance(task, tuple):  # Asegurarse de que task es una tupla
                        bot.getTasks(task)
                        bot.goal = task[0]  # Asignar la posición objetivo
                        bot.path = bot.a_star(bot.pos, bot.goal)

        # Asignar tareas a los RBot para que recojan paquetes de estantes
        for rbot in self.model.schedule.agents:
            if isinstance(rbot, RBot) and not rbot.carry and not rbot.goal and self.current_order:
                task = self.get_next_task_for_order()
                if task:
                    rbot.getTasksR(task)
                    rbot.goal = task[0]
                    rbot.path = rbot.a_star(rbot.pos, rbot.goal)

    def add_task(self, task):
        self.tasks.append(task)

    def create_new_order(self):
        """Crea un nuevo pedido personalizado cada vez que se completa uno."""
        self.custom_order = {
            "PackageA": random.randint(5, 10),
            "PackageB": random.randint(5, 10),
            "PackageC": random.randint(5, 10),
            "fulfilled": {
                "PackageA": 0,
                "PackageB": 0,
                "PackageC": 0
            }
        }
        print(f"Nuevo pedido creado: {self.custom_order}")

    
    def get_next_task_for_order(self):
        # Find which type of package is needed
        for package_type, count in self.current_order.items():
            if count > self.current_order["fulfilled"][package_type]:
                shelf_type = ShelfA if package_type == "PackageA" else ShelfB if package_type == "PackageB" else ShelfC
                # Find a shelf with this package type
                for pos in self.model.grid.coord_iter():
                    _, (x, y) = pos
                    shelf_at_pos = self.model.grid.get_cell_list_contents([x, y])
                    for obj in shelf_at_pos:
                        if isinstance(obj, shelf_type) and obj.has_capacity():
                            return (x, y), package_type
        return None
    
    def fulfill_order(self, package_type):
        """Actualiza el campo 'fulfilled' en el pedido personalizado."""
        if package_type in self.custom_order["fulfilled"]:
            self.custom_order["fulfilled"][package_type] += 1

        # Verificar si el pedido está completo
        complete = all(
            self.custom_order["fulfilled"][pkg] >= self.custom_order[pkg]
            for pkg in ["PackageA", "PackageB", "PackageC"]
        )

        if complete:
            print("El pedido se ha completado.")
            # Aquí podrías crear un nuevo pedido si es necesario
            self.create_new_order()

    def is_order_fulfilled(self, shelf_type):
        """Verifica si la parte de la orden correspondiente a un tipo de estante está completa."""
        if shelf_type == ShelfA:
            package_type = "PackageA"
        elif shelf_type == ShelfB:
            package_type = "PackageB"
        elif shelf_type == ShelfC:
            package_type = "PackageC"
        else:
            return False  # Si el tipo de estante no coincide, no está cumplido

        return self.custom_order["fulfilled"][package_type] >= self.custom_order[package_type]
    def add_shelf_task(self, task):
        """Agrega una tarea para recoger de un shelf, asegurándose de que no esté ya asignada."""
        if task not in self.shelf_tasks:  # Evitar duplicados
            self.shelf_tasks.append(task)

    def update_tasks(self):
        for pos in self.model.grid.coord_iter():
            _, agents = pos
            for agent in agents:
                if isinstance(agent, (PackageA, PackageB, PackageC)) and pos not in self.assigned_shelves:
                    task = (pos, None)
                    self.add_task(task)

    def update_shelf_tasks(self):
        for pos in self.model.grid.coord_iter():
            _, agents = pos
            for agent in agents:
                if isinstance(agent, (ShelfA, ShelfB, ShelfC)) and agent.capacity < 3 and pos not in self.assigned_shelves:
                    task = (pos, None)
                    self.add_shelf_task(task)
                    self.assigned_shelves[pos] = True

    def assign_task(self, rbot):
        """Asigna una tarea solo si el RBot no ha completado su parte del pedido."""
        shelf_type = rbot.shelf_type

        if self.is_order_fulfilled(shelf_type):
            print(f"RBot {rbot.unique_id} ya completó su parte del pedido.")
            return None

        # Filtrar tareas de shelf según el tipo de shelf que maneja el RBot y que tengan capacidad disponible
        shelf_tasks_filtered = [
            task for task in self.shelf_tasks 
            if isinstance(self.model.grid.get_cell_list_contents(task[0])[0], shelf_type)
            and self.model.grid.get_cell_list_contents(task[0])[0].capacity < 3  # Verificar capacidad
        ]

        if shelf_tasks_filtered:
            task = shelf_tasks_filtered.pop(0)  # Tomar la primera tarea disponible de ese tipo de shelf
            self.assigned_shelves[task[0]] = rbot.unique_id  # Registrar que el shelf está asignado a un RBot
            return task

    def complete_task(self, rbot):
        """Marca la tarea como completada y elimina la asignación del shelf."""
        # Eliminar el shelf de la lista de asignados
        for shelf_pos, bot_id in list(self.assigned_shelves.items()):
            if bot_id == rbot.unique_id:
                del self.assigned_shelves[shelf_pos]
                break

    def occupy_spackage(self, spackage_pos):
        """Marca un SPackage como ocupado."""
        self.occupied_spackages.add(spackage_pos)

    def release_spackage(self, spackage_pos):
        """Libera un SPackage una vez completado."""
        if spackage_pos in self.occupied_spackages:
            self.occupied_spackages.remove(spackage_pos)

    def is_spackage_occupied(self, spackage_pos):
        """Verifica si un SPackage está ocupado."""
        return spackage_pos in self.occupied_spackages

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
        self.timer = random.randint(30, 100)  # Temporizador para controlar el spawn de paquetes

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
        self.path = []  # Para almacenar la trayectoria
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
            "pickupPoints": [],
            "order": [],
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
            if isinstance(agent, Bot) and agent.next_pos ==  next_pos or isinstance(agent, RBot) and agent.next_pos == next_pos:
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
                self.goal = self.find_shelf(1 if self.isCarryingA else 2 if self.isCarryingB else 3)
                if self.goal:
                    self.path = self.a_star(self.pos, self.goal)

        if self.path and self.goal:
            self.next_pos = self.path.pop(0)
            if self.next_pos is None or self.model.grid.out_of_bounds(self.next_pos):
                return

            if self.detect_collision(self.next_pos):
                self.path = self.find_alternative_path(self.pos, self.goal)
                if not self.path:
                    return  
                self.next_pos = self.path.pop(0)
                if self.next_pos is None or self.model.grid.out_of_bounds(self.next_pos):
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
                self.model.packages_in_shelves += 1
                if isinstance(shelf, ShelfA) and self.isCarryingA and shelf.has_capacity():
                    shelf.decrement_capacity()
                    self.history["order"].append("A")
                    if not shelf.has_capacity():
                        shelf.is_full = True
                    self.isCarryingA = False
                elif isinstance(shelf, ShelfB) and self.isCarryingB and shelf.has_capacity():
                    shelf.decrement_capacity()
                    self.history["order"].append("B")
                    if not shelf.has_capacity():
                        shelf.is_full = True
                    self.isCarryingB = False
                elif isinstance(shelf, ShelfC) and self.isCarryingC and shelf.has_capacity():
                    shelf.decrement_capacity()
                    self.history["order"].append("C")
                    if not shelf.has_capacity():
                        shelf.is_full = True
                    self.isCarryingC = False
                else:
                    return

                self.carry = False

                self.goal = None
                self.path = []
                self.reset_carrying_flags()
                self.history["deliveryPoints"].append({"x": self.next_pos[0] + 1, "y": self.next_pos[1]})

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

class RBot(Agent):
    def __init__(self, unique_id, model, initial_position, shelf_type):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.path = []  # Para almacenar la trayectoria
        self.tasks = []
        self.carry = False
        self.goal = None
        self.initial_position = initial_position
        self.shelf_type = shelf_type
        self.isCarryingA = False
        self.isCarryingB = False
        self.isCarryingC = False
        self.movements = 0
        self.history = {
            "spawnPosition": {"x": initial_position[0], "y": initial_position[1]},
            "path": [],
            "deliveryPoints": [],
            "pickupPoints": [],
            "order": [],
        }
        self.returning_to_start = False
        self.completed_order = False

    def step(self):
        """Simula un paso del RBot."""
        # Registrar la posición actual en el historial
        self.history["path"].append({"x": self.pos[0], "y": self.pos[1]})

        # Si el RBot ha completado su parte del pedido, regresa a su posición inicial y espera
        if self.completed_order:
            if not self.goal:
                self.goal = self.initial_position
                self.path = self.a_star(self.pos, self.goal)
            elif self.path:
                self.next_pos = self.path.pop(0)
                self.model.grid.move_agent(self, self.next_pos)
            return

        # Si no lleva un paquete y no tiene un objetivo, asignar una tarea
        if not self.carry and not self.goal:
            task = self.model.central_system.assign_task(self)
            if task:
                self.goal = task[0]
                self.path = self.a_star(self.pos, self.goal)
            else:
                self.goal = None

        # Si el RBot tiene una tarea y no lleva un paquete, ejecuta la tarea
        if not self.carry:
            if self.path and len(self.path) > 0:
                self.next_pos = self.path.pop(0)

                # Verifica si hay colisión antes de moverse
                if self.detect_collision(self.next_pos):
                    self.path = self.find_alternative_path(self.pos, self.goal)
                    if not self.path:
                        return
                    self.next_pos = self.path.pop(0)

                # Mover el RBot si no hay colisión
                if self.next_pos:
                    self.model.grid.move_agent(self, self.next_pos)

                # Si está adyacente al shelf, recoger el paquete
                if self.is_adjacent(self.next_pos, self.goal):
                    shelf = self.get_shelf_at_position(self.goal)
                    if shelf:
                        self.pickup_from_shelf(shelf)
                        self.goal = self.find_available_spackage()
                        if self.goal:
                            self.path = self.a_star(self.pos, self.goal)

        else:
            # Si lleva un paquete, ir a dejarlo en el SPackage
            if self.path and len(self.path) > 0:
                self.next_pos = self.path.pop(0)

                # Verifica si hay colisión antes de moverse
                if self.detect_collision(self.next_pos):
                    self.path = self.find_alternative_path(self.pos, self.goal)
                    if not self.path:
                        return
                    self.next_pos = self.path.pop(0)

                # Mover el RBot si no hay colisión
                if self.next_pos:
                    self.model.grid.move_agent(self, self.next_pos)
                    self.movements += 1

                # Si está adyacente al SPackage, dejar el paquete
                if self.is_adjacent(self.next_pos, self.goal):
                    self.drop_off_package()
                    self.model.central_system.fulfill_order(self.get_package_type())
                    print(f"Pedido actualizado: {self.model.central_system.custom_order['fulfilled']}")
                    self.goal = None
                    self.path = []
                    
                    # Verificar si ha cumplido con su parte del pedido
                    if self.has_fulfilled_order():
                        print(f"RBot {self.unique_id} ha completado su parte del pedido.")
                        self.completed_order = True
                        self.return_to_initial_position()

    def get_package_type(self):
        """Determina el tipo de paquete que el RBot está llevando."""
        if self.isCarryingA:
            return "PackageA"
        elif self.isCarryingB:
            return "PackageB"
        elif self.isCarryingC:
            return "PackageC"
        return None

    def has_fulfilled_order(self):
        """Verifica si el RBot ha cumplido su parte de la orden."""
        package_type = self.get_package_type()
        if package_type:
            return self.model.central_system.custom_order["fulfilled"][package_type] >= self.model.central_system.custom_order[package_type]
        return False

    def return_to_initial_position(self):
        """Inicia el proceso para que el RBot regrese a su posición inicial."""
        self.goal = self.initial_position
        self.path = self.a_star(self.pos, self.goal)
        self.returning_to_start = True

    def pickup_from_shelf(self, shelf):
        if shelf.capacity < 3:
            shelf.increment_capacity()
            self.carry = True
            self.history["pickupPoints"].append({"x": self.next_pos[0], "y": self.next_pos[1]})
            if isinstance(shelf, ShelfA):
                self.isCarryingA = True
                self.history["order"].append("A")
            elif isinstance(shelf, ShelfB):
                self.isCarryingB = True
                self.history["order"].append("B")
            elif isinstance(shelf, ShelfC):
                self.isCarryingC = True
                self.history["order"].append("C")

    def drop_off_package(self):
        """Deja el paquete en el SPackage."""
        self.carry = False
        package_type = self.get_package_type()
        self.history["deliveryPoints"].append({"x": self.next_pos[0] + 1, "y": self.next_pos[1]})
        
        self.model.central_system.fulfill_order(package_type)
        self.model.central_system.release_spackage(self.goal)  # Liberar el SPackage

        self.isCarryingA = False
        self.isCarryingB = False
        self.isCarryingC = False


        self.model.packages_in_spackage += 1
        
        print(f"RBot {self.unique_id} dejó el paquete y está listo para otro.")


    def get_shelf_at_position(self, pos):
        """Obtiene el shelf en una posición dada."""
        contents = self.model.grid.get_cell_list_contents([pos])
        if contents:
            for obj in contents:
                if isinstance(obj, (ShelfA, ShelfB, ShelfC)):
                    return obj
        return None

    def find_available_spackage(self):
        """Encuentra un SPackage disponible para la entrega."""
        for pos in self.model.grid.coord_iter():
            _, (x, y) = pos
            contents = self.model.grid.get_cell_list_contents([x, y])
            if contents:
                for obj in contents:
                    if isinstance(obj, SPackage) and not self.model.central_system.is_spackage_occupied((x, y)):
                        self.model.central_system.occupy_spackage((x, y))  # Marcar SPackage como ocupado
                        return (x, y)
        return None

    def is_adjacent(self, pos1, pos2):
        """Verifica si dos posiciones son adyacentes."""
        if pos1 and pos2:
            return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]) == 1
        return False

    def detect_collision(self, next_pos):
        """Detecta colisiones con otros robots."""
        for agent in self.model.grid.get_neighbors(self.pos, moore=False, include_center=False, radius=1):
            if isinstance(agent, (Bot, RBot)) and agent.next_pos == next_pos:
                return True
        return False

    def find_alternative_path(self, start, goal):
        """Encuentra un camino alternativo evitando otros robots."""
        avoid_positions = [agent.next_pos for agent in self.model.schedule.agents if isinstance(agent, RBot) and agent.next_pos is not None]
        return self.a_star(start, goal, avoid_positions)

    def a_star(self, start, goal, avoid_positions=[]):
        """Implementa el algoritmo de búsqueda A* para encontrar el camino."""
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

    def euclidean_heuristic(self, pos, goal):
        """Calcula la heurística euclidiana para A*."""
        return np.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)

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
        'WFFFFFFLLLLFFFFFFUFFFFFFFFFFFFFFFFFF2W',
        'DFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF2W',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF1W',
        'DFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF2W',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFF2W',
        'DFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WUFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFF3W',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WFFFFFFFCCFFFFFFFFBBFFFFFFFFAAFFFFFFFW',
        'WLFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFLW',
        'WLFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFLW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFFFFRFFRFFRFFFFFFFFTFFTFFTFFFFFFFW',
        'W3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFUW',
        'WFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFW',
        'WFFFFLLLLLFFFFFF22F1F22FFFFFFFLLLLFFFW',
        'WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW',
    ]


        self._manual_placement(self.desc, bot_id)

        self.packages_in_shelves = 0
        self.packages_in_spackage = 0

        self.datacollector = DataCollector(
            model_reporters={
                "TotalMovements": lambda m: int(np.sum([a.movements for a in m.schedule.agents if isinstance(a, (Bot, RBot))])),
                "PackagesInShelves": lambda m: m.packages_in_shelves,
                "PackagesInSPackage": lambda m: m.packages_in_spackage,
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
                elif desc[y][x] == 'L':  
                    plant = Plant(self.next_id(), self)
                    self.grid.place_agent(plant, (x, y))
                elif desc[y][x] == '1':  
                    cafe1 = Cafe1(self.next_id(), self)
                    self.grid.place_agent(cafe1, (x, y))
                elif desc[y][x] == '2':  
                    cafe2 = Cafe2(self.next_id(), self)
                    self.grid.place_agent(cafe2, (x, y))
                elif desc[y][x] == '3':  
                    ext = Extintor(self.next_id(), self)
                    self.grid.place_agent(ext, (x, y))
                elif desc[y][x] == 'U':  
                    inclusion = Inclusion(self.next_id(), self)
                    self.grid.place_agent(inclusion, (x, y))
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
                elif desc[y][x] == 'T':
                    if bot_id == 4:  # Primer RBot trabaja con ShelfA
                        rbot = RBot(bot_id, self, (x, y), ShelfA)
                    elif bot_id == 5:  # Segundo RBot trabaja con ShelfB
                        rbot = RBot(bot_id, self, (x, y), ShelfB)
                    else:  # Tercer RBot trabaja con ShelfC
                        rbot = RBot(bot_id, self, (x, y), ShelfC)
                    
                    rbot.initial_position = (x, y)
                    self.grid.place_agent(rbot, (x, y))
                    self.schedule.add(rbot)
                    bot_id += 1

        for box_pos in box_position:
            self.central_system.add_task((box_pos, goalPos))

    def generate_simulation_report(self):
        report = {"robots": []}

        for agent in self.schedule.agents:
            if isinstance(agent, (Bot, RBot)):
                report["robots"].append({
                    "spawnPosition": {"x": agent.initial_position[0], "y": agent.initial_position[1]},
                    "path": agent.path,
                    "deliveryPoints": agent.delivery_points,
                    "pickupPoints": agent.pickup_points,
                    "order": agent.order
                })

        with open("simulation_report.json", "w") as json_file:
            json.dump(report, json_file, indent=4)

    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()

        # Save the current state of all bots to JSON
        robots_data = {"robots": []}
        for agent in self.schedule.agents:
            if isinstance(agent, Bot) or isinstance(agent, RBot):
                robots_data["robots"].append(agent.history)

        with open("resultados_simulacion.json", "w") as json_file:
            json.dump(robots_data, json_file, indent=4)

        self.running = any([isinstance(a, Bot) for a in self.schedule.agents])


