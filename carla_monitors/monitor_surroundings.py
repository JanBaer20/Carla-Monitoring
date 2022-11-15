import py_trees
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion
from srunner.scenariomanager.timer import GameTime

from carla_helpers.world_helper import WorldHelper


class MonitorSurroundings(Criterion):
    """
    This class contains the monitor to output the surrounding world
    """

    MAX_IDLE_TIME = 1  # Amount of time in seconds until a vehicle is flagged as "stopped"
    MIN_VELOCITY = 0.5  # Amount of speed necessary to start the test

    def __init__(self, actor, world, debug_mode, name="MonitorSurroundings", terminate_on_failure=False):
        super(MonitorSurroundings, self).__init__(name, actor, None,
                                                  terminate_on_failure=terminate_on_failure)

        self.logger.debug("%s.__init__()" % self.__class__.__name__)

        self._actor = self.actor
        self._debug_mode = debug_mode
        self._world = world
        self._world_helper = WorldHelper(world)

    # Gets called each tick and checks the implemented test
    def update(self):
        """
        Check velocity == 0
        :return: Current status of the test
        """
        if self._debug_mode:
            print(f"Current time: {GameTime.get_time()}")

        traffic_lights = self._world_helper.get_traffic_lights_for_vehicle(self._actor)
        if self._debug_mode:
            print(f"Traffic Lights: ", traffic_lights)
            for traffic_light in traffic_lights:
                print(f"Traffic Light:", traffic_light.id, traffic_light.state)

        # Saves the status of the current update cycle of the VehicleStoppedTest
        new_test_status = py_trees.common.Status.RUNNING

        return new_test_status
