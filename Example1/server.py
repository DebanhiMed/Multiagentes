import mesa
from model import Environment, Bot, Shelf, Goal, Wall, Charger, SPackage, EPackage

def agent_portrayal(agent):
    if isinstance(agent, Bot):
        return {"Shape": "circle", "Filled": "false", "Color": "Cyan", "Layer": 1, "r": 0.9,
                "text": f"{agent.battery}", "text_color": "black"}
    elif isinstance(agent, Shelf):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "🫦"}
    elif isinstance(agent, Wall):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#000000", "text": ""}
    elif isinstance(agent, Goal):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "🎁"}
    elif isinstance(agent, Charger):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "🪫"}
    elif isinstance(agent, EPackage):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "📦"}
    elif isinstance(agent, SPackage):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "🚚"}
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

server = mesa.visualization.ModularServer(
    Environment, [grid, chart_charges],
    "Avance Bueno", model_params, 8521
)

server.launch(open_browser=True)