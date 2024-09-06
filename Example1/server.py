import mesa
from model import Environment, Bot, ShelfA, ShelfB, ShelfC, Wall, SPackage, PackageA, PackageB, PackageC, EPackage, RBot, Plant, Cafe1, Cafe2, Inclusion, Extintor

def agent_portrayal(agent):
    if isinstance(agent, Bot):
        return {"Shape": "circle", "Filled": "false", "Color": "peru", "Layer": 1, "r": 0.9,
                "text": "", "text_color": "black"}
    elif isinstance(agent, ShelfA):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "blanchedalmond", "text": f"{agent.capacity}"}
    #antiquewhite
    #burlywood
    #blanchedalmod
    elif isinstance(agent, ShelfB):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "wheat", "text": f"{agent.capacity}"}
    elif isinstance(agent, ShelfC):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "moccasin", "text": f"{agent.capacity}"}
    elif isinstance(agent, Wall):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#000000", "text": ""}
    elif isinstance(agent, Plant):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "White",
                "Color": "#FFFFFF", "text": "🪴"}
    elif isinstance(agent, Cafe1):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "White",
                "Color": "#FFFFFF", "text": "☕️"}
    elif isinstance(agent, Cafe2):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "White",
                "Color": "#FFFFFF", "text": "❗️"}
    elif isinstance(agent, Inclusion):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "White",
                "Color": "lightsteelblue", "text": "♿︎"}
    elif isinstance(agent, Extintor):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "White",
                "Color": "peachpuff", "text": "🧯"}
    elif isinstance(agent, PackageA):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "blanchedalmond", "text": "📦"}
    elif isinstance(agent, PackageB):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "wheat", "text": "📦"}
    elif isinstance(agent, PackageC):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "moccasin", "text": "📦"}
    elif isinstance(agent, SPackage):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "orangered", "text": "🚚"}
    elif isinstance(agent, EPackage):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "lime", "text": "🚛"}
    elif isinstance(agent, RBot):
        return {"Shape": "circle", "Filled": "false", "Color": "saddlebrown", "Layer": 1, "r": 0.9,
                "text": "", "text_color": "black"}
    else:
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "white", "text": ""}


grid = mesa.visualization.CanvasGrid(
    agent_portrayal, 38, 22, 600, 400)

# Create a chart to track the battery of the robots
chart = mesa.visualization.ChartModule(
    [
        {"Label": "TotalMovements", "Color": "#00AA00"},
        {"Label": "PackagesInShelves", "Color": "#FF0000"},
        {"Label": "PackagesInSPackage", "Color": "#0000FF"},
    ],
    data_collector_name='datacollector'
)

model_params = {
    "M": 38,
    "N": 22,
}

# Inicializar el servidor y el modelo
server = mesa.visualization.ModularServer(
    Environment, [grid, chart], "Warehouse Simulation", model_params, 8521
)

# Lanzar el servidor y la simulación
server.launch(open_browser=True)

# Ejecutar la simulación y guardar los datos al finalizar
model = server.model
while model.running:
    model.step()

# Guardar los datos al finalizar la simulación
model.save_data_to_json("resultados_simulacion.json")
