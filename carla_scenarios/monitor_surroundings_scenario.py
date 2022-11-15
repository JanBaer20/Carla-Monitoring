import py_trees
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenarios.basic_scenario import BasicScenario

from carla_helpers.world_helper import WorldHelper
from carla_monitors.monitor_ego_vehicle import MonitorEgoVehicle


class MonitorSurroundingsScenario(BasicScenario):
    """
    This class build the scenario in which the ego vehicles just follows the waypoints until the timeout runs out and
    simultaneously tracks all data surrounding the vehicle
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=True, criteria_enable=True,
                 timeout=6000):
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
        self.world_helper = WorldHelper(world)

        super(MonitorSurroundingsScenario, self).__init__("MonitorSurroundingsScenario.xml", ego_vehicles, config,
                                                          world, randomize,
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
        for ego_vehicle in self.ego_vehicles:
            collision_criterion = MonitorEgoVehicle(ego_vehicle, self._world, debug_mode=False)
            criteria.append(collision_criterion)
        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
