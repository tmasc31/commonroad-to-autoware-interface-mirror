"""
What does the state machine planner actually do?
- self._get_departing_coord_system() # only for bay scenario, chosen based on vel
- planner, config = self._create_planner() # reactive planner (adapted)
- planner.set_desired_velocity() or planner.set_desired_lon_position() (when stopping)
- next_state = self._plan_and_optimize()
- change state (of state machine) if conditions met (head -> arrive, arrive -> stop/before stop, before stop -> stop; stop -> depart is done in step())
- state transitions are done via distance threshold/stopped counter checks!
- next_step (what step() returns) is a CR state! not a state machine state!


state machine planner does not output an explicit lanelet sequence/route generator.
It just provides the correct coordinate system, which contains a reference path
-->> So, set the reference path for CR2AW to that of the state machine coord sys!!
	- implement plan_routes() and update_planning_problem_and_plan_routes() [done]
	- replace CR planner with state machine! [done]
	- remove reactive planner creation from state machine, publish/output data properly
    - skip lanelet representation and go straight to reference path [done]
    - implement state transitions so that new reference paths are generated
	- bypass velocity smoother, pass desired velocity directly to trajectory planner either:
        - modified velocity planner that gets reference velocity from state yamls or state machine (respects interface)
        - bypass velocity planner entirely and get the reference velocity from the route planner (less code)
"""

import logging
import numpy as np
import os
from pathlib import Path
import yaml

from obacht.post_optimization_planner.state_machine_planner import BaseStateMachinePlanner
from obacht.post_optimization_planner.state_machine_planner import create_state_machine_planner
from obacht.source.planner.iterator import scenario_iterator_interactive
from obacht.source.simulation.simulations import load_sumo_configuration

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem

from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)
from commonroad_route_planner.route_generation_strategies.default_generation_strategy import (
    DefaultGenerationStrategy,
)

from typing import List, Union


class ObachtRoutePlannerWrapper:
    def __init__(self, **kwargs):
        """
        Initialization of the Obacht Planner Wrapper
        (Matches the interface of RoutePlanner from commonroad_route_planner).
        """

        scenario_dir = "/commonroad/scenarios"
        scenario_path = next(scenario_iterator_interactive(scenario_dir))
        yaml_path = Path(scenario_path).parent.parent/"configurations" / "scenario.yaml"
        conf = load_sumo_configuration(scenario_path)
        scenario_file = os.path.join(scenario_path, f"{conf.scenario_name}.cr.xml")

        # FIXME: pass planning problem through ROS topic, not directly in constructor
        self._scenario, self.planning_problem = CommonRoadFileReader(scenario_file).open()

        try:
            with open(yaml_path, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)
                scenario_type = config.get('scenario', {}).get('type')
        except FileNotFoundError:
            print(f"file: {yaml_path} not found")
            return None
        except yaml.YAMLError as e:
            print(f"YAML error: {e}")
            return None

        self._planner: BaseStateMachinePlanner = create_state_machine_planner(
            self.scenario,
            self.planning_problem,
            scenario_type,
            use_post_opt=False,
            initial_state_name="HEADING",
        )
            
    
    def update_planning_problem_and_plan_routes(self, **kwargs) -> np.ndarray:
        # ignore because we are providing our own planning problem from the scenario file
        return self.plan_routes()
    
    def plan_routes(self) -> np.ndarray:
        """
        Plans routes for every pair of start/goal lanelets. If no goal lanelet ID is given then return a survival route.

        :param lane_change_method: Method for lane changes, e.g. quintic splines
        :param GenerationStrategy: generation strategy for route

        :return: reference path
        """

        return self._planner.coord_systems[self._planner.current_state].reference