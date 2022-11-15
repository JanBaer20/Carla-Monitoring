import py_trees
from carla_monitors.vehicle_over_speedlimit import VehicleOverSpeedlimit
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenarios.basic_scenario import BasicScenario

from carla_monitors.vehicle_stopped_test import VehicleStoppedTest
from carla_monitors.vehicle_data_test import VehicleDataTest
from carla_monitors.vehicle_over_speedlimit import VehicleOverSpeedlimit
from carla_monitors.vehicle_passed_red_traffic_light import VehiclePassedRedTrafficLightTest


class VehicleStoppedScenario(BasicScenario):
    """
    This class holds the monitor to check whether the ego vehicle has stopped.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
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
        self._world = world

        super(VehicleStoppedScenario, self).__init__("VehicleStoppedScenario", ego_vehicles, config, world, randomize,
                                                     debug_mode, criteria_enable)

    def _setup_scenario_trigger(self, config):
        return None

    def _create_behavior(self):
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(Idle())
        return sequence

    def _create_test_criteria(self):
        """
        This scenarios test if the ego vehicle stopped
        :return: List of criteria to check
        """
        criteria = []
        # Initialize the VehicleStoppedTest
        #stopped_vehicle_test = VehicleStoppedTest(self.ego_vehicles[0], debug_mode=self._debug_mode,
        #                                          terminate_on_failure=False)
        # stopped_vehicle_test = VehicleDataTest(self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
        #                                          terminate_on_failure=False)
        #stopped_vehicle_test = VehicleOverSpeedlimit(self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
        #                                         terminate_on_failure=False)
        #red_traffic_light = VehiclePassedRedTrafficLightTest(self.ego_vehicles[0], debug_mode=self._debug_mode,
        #                                          terminate_on_failure=False)
        # Append to list so that it can be returned later...
        #criteria.append(stopped_vehicle_test)
        #criteria.append(red_traffic_light)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
