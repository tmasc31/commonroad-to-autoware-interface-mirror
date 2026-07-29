import copy
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
    """
    This class acts as an intermediate between the main cr2autoware.py script and
    the OBACHT Route Planner. It implements the functions cr2autoware.py expects
    from a Route Planner, implementing the required functions so that the existing
    higher level planner wrapper CommonRoadRoutePlanner (cr_route_planner.py) can use it
    (instead of the RoutePlanner from the common-road-route-planner package). It sets up
    the route planner initial state with the scenario data from the scenario configured
    in obacht/configurations/scenario.yaml.
    """

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

        self.state_list = [copy.deepcopy(self.planning_problem.initial_state)]
        self.state_list[0].time_step = 0

        self._planner: BaseStateMachinePlanner = create_state_machine_planner(
            self.scenario,
            self.planning_problem,
            scenario_type,
            use_post_opt=False,
            initial_state_name="HEADING",
        )
        self.current_state = "HEADING"
            
    
    def update_planning_problem_and_plan_routes(self, planning_problem, **kwargs) -> np.ndarray:
        # ignore because we are providing our own planning problem from the scenario file
        return self.plan_routes(**kwargs)
    
    def plan_routes(self, ego_vehicle_state, **kwargs) -> np.ndarray:
        """
        Generates a reference path for the path planning problem given the
        current vehicle state.

        :param ego_vehicle_state: Current vehicle state

        :return: reference path, desired velocity for current state machine state
        """

        # pass current state and apply state transitions as needed
        # FIXME: this is called every time a new goal position is set,
        # so desired velocity is not updated as the state machine evolves!
        self.current_state = self._planner.current_state
        self.next_state = self._planner.step(ego_vehicle_state, self.state_list)

        return self._planner.coord_systems[self._planner.current_state].reference, self._planner.desired_velocity
