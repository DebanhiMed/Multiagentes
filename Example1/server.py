import mesa
from model import Environment, Bot, Shelf, Goal, Wall

MAX_NUMBER_ROBOTS = 20
MAX_NUMBER_GOALS = 50  # Set a maximum for the number of goal

def agent_portrayal(agent):
    if isinstance(agent, Bot):
        return {"Shape": "circle", "Filled": "false", "Color": "Cyan", "Layer": 1, "r": 0.9,
                "text": f"{agent.battery}", "text_color": "black"}
    elif isinstance(agent, Shelf):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "🫦"}
    elif isinstance(agent, Wall):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "█"}
    elif isinstance(agent, Goal):
        return {"Shape": "rect", "Filled": "false", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "🎁"}
    else:
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "white", "text": ""}

grid = mesa.visualization.CanvasGrid(
    agent_portrayal, 19, 11, 400, 300)

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
    "num_agents": mesa.visualization.Slider(
        "Number of Robots",
        5,
        2,
        MAX_NUMBER_ROBOTS,
        1,
        description="Choose how many robots to include in the model",
    ),
    "num_goals": mesa.visualization.Slider(
        "Number of Goals",
        1,
        0,
        MAX_NUMBER_GOALS,
        1,
        description="Choose the number of goals to include in the model",
    ),
    "obstacle_portion": mesa.visualization.Slider(
        "Obstacle Portion",
        0.3,
        0.0,
        0.75,
        0.05,
        description="Choose the percentage of obstacles in the environment",
    ),
    "mode_start_pos": mesa.visualization.Choice(
        "Bot Start Position Mode",
        "Random",
        ["Fixed", "Random"],
        "Choose whether to place the robots in a fixed position or randomly",
    ),
    "M": 19,
    "N": 11,
}

server = mesa.visualization.ModularServer(
    Environment, [grid, chart_charges],
    "Avance 3 Prueba amo a debanhi 😍", model_params, 8521
)

server.launch(open_browser=True)
