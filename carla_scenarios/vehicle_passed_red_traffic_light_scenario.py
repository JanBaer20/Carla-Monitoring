import py_trees
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, WaypointFollower
from srunner.scenarios.basic_scenario import BasicScenario

from carla_monitors.vehicle_passed_red_traffic_light import VehiclePassedRedTrafficLightTest
from carla_monitors.vehicle_stopped_test import VehicleStoppedTest


class VehiclePassedRedTrafficLightScenario(BasicScenario):
    """
    This class holds the monitor to check whether the ego vehicle has passed a red light.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=30):
        """
        Initialize all parameters required for this scenario
        :param world: The world with all its data
        :param ego_vehicles: List of all ego vehicles
        :param config: Configurations passed as config params
        :param randomize: Whether the scenario should be randomized
        :param debug_mode: Whether to use the debug mode and print out debug information
        :param criteria_enable:
        """

        # Apparently the class needs this attribute to be loaded by the ScenarioLoader
        self.timeout = timeout
        self._debug_mode = debug_mode

        super(VehiclePassedRedTrafficLightScenario, self).__init__("VehiclePassedRedTrafficLightScenario", ego_vehicles,
                                                                   config, world, randomize,
                                                                   debug_mode, criteria_enable)

    def _setup_scenario_trigger(self, config):
        return None

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence("Sequence Behavior")

        # Create behavior in which the given vehicle follows the waypoints in front
        follow_waypoints = WaypointFollower(self.ego_vehicles[0], 30)

        sequence.add_child(follow_waypoints)
        return sequence

    def _create_test_criteria(self):
        """
        This scenarios test if the ego vehicle stopped
        :return: List of criteria to check
        """
        criteria = []
        # Initialize the VehicleStoppedTest
        passed_red = VehiclePassedRedTrafficLightTest(self.ego_vehicles[0], debug_mode=self._debug_mode,
                                                      terminate_on_failure=False)
        # Append to list so that it can be returned later...
        criteria.append(passed_red)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
