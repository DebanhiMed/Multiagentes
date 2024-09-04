import mesa
from model import Environment, Bot, ShelfA, ShelfB, ShelfC, Wall, SPackage, PackageA, PackageB, PackageC, EPackage, RBot

def agent_portrayal(agent):
    if isinstance(agent, Bot):
        return {"Shape": "circle", "Filled": "false", "Color": "Grey", "Layer": 1, "r": 0.9,
                "text": "", "text_color": "black"}
    elif isinstance(agent, ShelfA):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#FFB200", "text": f"{agent.capacity}"}
    elif isinstance(agent, ShelfB):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#FFCB42", "text": f"{agent.capacity}"}
    elif isinstance(agent, ShelfC):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#FFC54D", "text": f"{agent.capacity}"}
    elif isinstance(agent, Wall):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#000000", "text": ""}
    elif isinstance(agent, PackageA):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#FFB200", "text": "📦"}
    elif isinstance(agent, PackageB):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#FFCB42", "text": "📦"}
    elif isinstance(agent, PackageC):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#FFC54D", "text": "📦"}
    elif isinstance(agent, SPackage):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "🚚"}
    elif isinstance(agent, EPackage):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeff", "text": "🚛"}
    elif isinstance(agent, RBot):
        return {"Shape": "circle", "Filled": "true", "Color": "blue", "Layer": 1, "r": 0.9,
                "text": "R", "text_color": "white"}
    else:
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "white", "text": ""}

grid = mesa.visualization.CanvasGrid(
    agent_portrayal, 38, 22, 600, 400)

# Create a chart to track the battery of the robots
chart_charges = mesa.visualization.ChartModule(
    [
        {"Label": "MinBattery", "Color": "#DD3B30", "label": "Min Battery"},
        {"Label": "MaxBattery", "Color": "#403EDD", "label": "Max Battery"},
        {"Label": "MeanBattery", "Color": "#DD1BD7", "label": "Mean Battery"},
    ],
    data_collector_name='datacollector'
)

model_params = {
    "M": 38,
    "N": 22,
}

# Inicializar el servidor y el modelo
server = mesa.visualization.ModularServer(
    Environment, [grid, chart_charges],
    "goats", model_params, 8521
)

# Lanzar el servidor y la simulación
server.launch(open_browser=True)

# Ejecutar la simulación y guardar los datos al finalizar
model = server.model
while model.running:
    model.step()

# Guardar los datos al finalizar la simulación
model.save_data_to_json("resultados_simulacion.json")
