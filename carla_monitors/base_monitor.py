import os
import os.path
import json
from abc import abstractmethod
from datetime import datetime
from os.path import dirname

import pendulum as pendulum
import py_trees
from carla import World
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion
from srunner.scenariomanager.timer import GameTime

from carla_data_classes.DataBlock import DataBlock
from carla_data_classes.DataTick import DataTick
from carla_data_classes.EnhancedJSONEncoder import EnhancedJSONEncoder
from carla_helpers.world_helper import WorldHelper


class BaseMonitor(Criterion):
    """
    This class contains the base monitor with logic necessary for all monitors
    """

    # Sets the time in seconds after how many ticks a log entry should be created
    TICK_LOG_TIME = 0.5  # seconds

    def __init__(self, actor, world, debug_mode, name="BaseMonitor", terminate_on_failure=False):
        super(BaseMonitor, self).__init__(name, actor, None,
                                          terminate_on_failure=terminate_on_failure)

        self.logger.debug("%s.__init__()" % self.__class__.__name__)

        self._actor = self.actor
        self._debug_mode = debug_mode
        self._world: World = world
        self._world_helper = WorldHelper(world)
        self._last_tracked_tick = GameTime.get_time()
        # Container for existing data in log file
        self._existing_data = []
        self._initialized_datetime = datetime.now(pendulum.timezone('CET'))
        self.create_and_read_log(name=name, read_data=True)

    def get_log_file_path(self, name: str):
        """
        Return the path for the log file with the given name
        :param name: The name of the log file
        :return: os.path for the log file
        """
        # Get parent directory of this file
        parent_directory = dirname(dirname(os.path.abspath(__file__)))
        # Path where logs folder should go
        log_directory = os.path.join(parent_directory, "logs")
        # Create folder if it does not exist
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        # Create path for log file
        cet = pendulum.timezone('CET')
        name_string = f"{name}_{self._initialized_datetime}".replace(":", "-").replace(" ", "_").replace(".", "_")
        return os.path.join(log_directory, f"{name_string}.json")

    def create_and_read_log(self, name: str, read_data: bool = False) -> None:
        """
        Creates the log file if not existing and returns its data if not empty
        :param name: The name of the log file that should be created/read
        :param read_data: Decides whether the data should be read and stored in self._existing_data
        :return: -
        """
        log_file_path = self.get_log_file_path(name)
        # Open log file and create if not existing
        with open(log_file_path, "w+") as logfile:
            # Get content of log file
            file_data = logfile.read()
            # Check if there is already data
            if file_data != '' and read_data:
                # Save existing data
                self._existing_data = json.loads(logfile.read())

    @abstractmethod
    def update(self) -> py_trees.common.Status:
        """
        Updates the py_trees status
        :return: The updates py_trees status
        """
        return py_trees.common.Status.RUNNING

    def calculate_tick(self) -> bool:
        """
        Returns whether the current tick should be calculated
        (If in the last TICK_LOG_TIME a tick was already calculated: skip
        :return: Whether the current tick should be calculated
        """
        time_diff_since_last_tracked_tick = GameTime.get_time() - self._last_tracked_tick
        # Check whether the TICK_LOG_TIME has passed since last tracked tick
        if time_diff_since_last_tracked_tick <= self.TICK_LOG_TIME:
            # Don't track current tick
            return False
        return True

    def log(self, data_obj: DataBlock = None, scenario_name="") -> None:
        """
        Log the given data in the log file with the name {self.name}_{scenario_name}
        :param data_obj: The DataBlock that should be logged
        :param scenario_name: The additional scenario_name (if desired)
        :return: None
        """
        current_tick = round(GameTime.get_time(), 2)
        print(f"Current Tick: {current_tick}")
        data_tick = DataTick(tick=current_tick, data=data_obj)
        # Add new json object to list of data already in log file
        self._existing_data.append(data_tick)
        logfile_name = self.get_log_file_path(f"{self.name}_{scenario_name}")
        self.log_data(data_tick, logfile_name)
        self._last_tracked_tick = current_tick

    def log_data(self, data: object, name: str) -> None:
        """
        Log arbitrary data into a log file with the given name
        :param data: The data to be logged
        :param name: The name of the log file to log into
        :return: None
        """
        # Override existing log entries with complete data list
        with open(name, "a") as logfile:
            json.dump(data, logfile, cls=EnhancedJSONEncoder)
