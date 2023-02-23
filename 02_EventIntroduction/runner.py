import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ""))
sys.path.append(os.path.join(os.path.dirname(__file__), "leaderboard"))
sys.path.append(os.path.join(os.path.dirname(__file__), "carla"))
sys.path.append(os.path.join(os.path.dirname(__file__), "carla\dist\carla-0.9.11-py3.7-win-amd64.egg"))
sys.path.append(os.path.join(os.path.dirname(__file__), "scenario_runner"))
sys.path.append(os.path.join(os.path.dirname(__file__), "tracks"))

from leaderboard.leaderboard_evaluator import main

routes = os.path.realpath(r"tracks\track_01.xml")
scenarios = os.path.realpath(r"tracks\track_01.json")

agent = os.path.realpath(r"src\agent.py")
agent_config = ""
debug = 0

main(routes, scenarios, agent, agent_config, debug)
