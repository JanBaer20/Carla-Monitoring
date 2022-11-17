import logging
from typing import List

import carla
import py_trees
from agents.navigation.local_planner import RoadOption
from py_trees.behaviour import Behaviour
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower, StopVehicle, Idle, ActorDestroy, LaneChange, TrafficLightStateSetter
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation, \
    WaitEndIntersection, InTriggerDistanceToVehicle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ReachedRegionTest, Criterion
from srunner.scenarios.basic_scenario import BasicScenario

from carla_helpers import ScenarioHelper
from carla_monitors.monitor_ego_vehicle import MonitorEgoVehicle, NoiseData, NoiseTargetEnum
# Initialization of logging
from carla_noise_generator.normal_noise_generator import NoiseTypeEnum

logger = logging.getLogger(__name__)
logging.basicConfig(format='%(module)s - %(levelname)s: %(message)s', level=logging.INFO)


class PruefungCityScenario(BasicScenario):
    """
    This class models the scenario city where the ego vehicle drives through the city.

    Starting scenario with:
    $ python3 /opt/scenario_runner/scenario_runner.py --scenario PruefungCityScenario --configFile="PruefungCityScenario.xml" --additionalScenario="pruefung_city_scenario.py" --reloadWorld --debug
    """

    def __init__(self, world, ego_vehicles, config, terminate_on_failure=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
           Init function for the class PruefungCityScenario

        Args:
            world (): the simulated world from carla
            ego_vehicles ():  the list of ego vehicles -> usually one entry
            config (): the current config of the simulation
            debug_mode (): enable debug output
            terminate_on_failure (): set termination of scenario in presents of failures
            criteria_enable ():  enable the evaluation of criteria for this scenario
            timeout ():  the timeout duration in sec.
       """
        self._ai_controllers = []
        self.timeout = timeout
        self._debug_mode = debug_mode
        self._map = world.get_map()
        self._world = world
        self.scenario_helper = ScenarioHelper(self._map)
        self.scenario_name = "PruefungCityScenario"
        self.subtype = config.subtype

        # Call constructor of BasicScenario
        super(PruefungCityScenario, self).__init__(
            self.scenario_name,
            ego_vehicles,
            config,
            world,
            terminate_on_failure=terminate_on_failure,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable)

        # set a new position for spectator
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(12.56666, 140.843, 110),
                                                carla.Rotation(pitch=-90, yaw=0)))

    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        if config.other_actors:
            new_actors = CarlaDataProvider.request_new_actors(config.other_actors)
            if not new_actors:
                raise Exception("Error: Unable to add actors")

            for new_actor in new_actors:
                new_actor.set_simulate_physics(False)
                self.other_actors.append(new_actor)

    def _create_behavior(self) -> Behaviour:
        """
        This method setups the behavior tree that contains the behavior of all non-ego vehicles during the scenario.

        Returns:
            Behaviour: the root node of the behavior tree
        """
        root = py_trees.composites.Parallel("alle actors")

        root.add_child(self.get_others_behavior())

        return root

    def _create_test_criteria(self) -> List[Criterion]:
        """
        This method should setup a list with all evaluation criteria and monitors for the scenario.

        Returns:
             List[Criterion]: List of criterions and monitors for this scenario
        """
        # final location for scneario end
        # unfreeze the vehicles to start moving but maintain start position throughout initialization
        criteria = []
        self.unfreezeVehicles()
        return criteria


    def get_others_behavior(self) -> Behaviour:
        """
    The behavior of all other actors

        Returns:
            Behaviour: The py_tree node for the behavior of the other actors
    """
        # the composite root for the other actors
        others_node = py_trees.composites.Parallel("AllOtheActors")

        # behavior of vehicle 1
        t2_sequence = py_trees.composites.Sequence("Ford_Mustang_Sequence")
        others_node.add_child(t2_sequence)
        veh1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh1")
        t2_sequence.add_child(WaypointFollower(veh1, 16,
                                            self.scenario_helper.get_waypoint_plan(locations=[
                                                (carla.Location(154.38, -184.39, 0), RoadOption.STRAIGHT),
                                                (carla.Location(155.21, -187.99, 0), RoadOption.STRAIGHT),
                                                (carla.Location(157.96, -191.29, 0), RoadOption.STRAIGHT),
                                                (carla.Location(162.07, -193.13, 0), RoadOption.STRAIGHT),
                                                (carla.Location(167.67, -193.95, 0), RoadOption.STRAIGHT),
                                                (carla.Location(181.17, -193.73, 0), RoadOption.STRAIGHT),
                                                (carla.Location(191.49, -193.6, 0), RoadOption.STRAIGHT),
                                                (carla.Location(202.75, -193.58, 0), RoadOption.STRAIGHT),
                                                (carla.Location(211.43, -191.22, 0), RoadOption.STRAIGHT),
                                                (carla.Location(219.64, -185.3, 0), RoadOption.STRAIGHT),
                                                (carla.Location(225.29, -177.46, 0), RoadOption.STRAIGHT),
                                                (carla.Location(231.43, -160.54, 0), RoadOption.STRAIGHT),
                                                (carla.Location(233.99, -147.51, 0), RoadOption.STRAIGHT),
                                                (carla.Location(234.68, -134.01, 0), RoadOption.STRAIGHT),
                                                (carla.Location(234.63, -121.29, 0), RoadOption.STRAIGHT),
                                                (carla.Location(234.4, -109.3, 0), RoadOption.STRAIGHT),
                                                (carla.Location(233.98, -96.9, 0), RoadOption.STRAIGHT),
                                                (carla.Location(233.5, -82.68, 0), RoadOption.STRAIGHT),
                                                (carla.Location(233.14, -68.1, 0), RoadOption.STRAIGHT),
                                                (carla.Location(232.71, -53.48, 0), RoadOption.STRAIGHT),
                                                (carla.Location(232.29, -37.29, 0), RoadOption.STRAIGHT),
                                                (carla.Location(231.96, -23.04, 0), RoadOption.STRAIGHT),
                                                (carla.Location(231.97, -13.81, 0), RoadOption.STRAIGHT),
                                                (carla.Location(231.57, -11.34, 0), RoadOption.STRAIGHT),
                                                (carla.Location(230.01, -8.36, 0), RoadOption.STRAIGHT),
                                                (carla.Location(225.67, -5.63, 0), RoadOption.STRAIGHT),
                                                (carla.Location(219.86, -5.31, 0), RoadOption.STRAIGHT),
                                                (carla.Location(213.26, -5.54, 0), RoadOption.STRAIGHT),
                                                (carla.Location(205.66, -5.53, 0), RoadOption.STRAIGHT),
                                                (carla.Location(198.18, -5.47, 0), RoadOption.STRAIGHT),
                                                (carla.Location(188.16, -5.62, 0), RoadOption.STRAIGHT)]),
                                            avoid_collision=True))
        t2_sequence.add_child(ActorDestroy(veh1))

        # behavior of vehicle 2
        citreon_sequence = py_trees.composites.Sequence("Citreon_Sequence")
        others_node.add_child(citreon_sequence)
        veh2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh2")
        citreon_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(232.7, -46.5, 0), 4.0))
        citreon_sequence.add_child(WaypointFollower(veh2, 15,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(198.07, 9.09, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(199.93, 9.15, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(205.59, 9.28, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(212.87, 9.38, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(222.59, 10.25, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(229.34, 15.02, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(231.01, 21.55, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(230.85, 27.77, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(230.72, 34.02, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(230.64, 36.1, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(StopVehicle(veh2, 1))
        citreon_sequence.add_child(Idle(7))
        citreon_sequence.add_child(WaypointFollower(veh2, 25,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(229.96, 47.69, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(229.85, 51.15, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(229.69, 59.62, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(229.61, 72.23, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(229.46, 87.04, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(229.26, 104.47, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(228.75, 119.59, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(228.28, 136.81, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(226.46, 151.18, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(221.23, 162.19, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(211.35, 173.46, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(200.33, 181.26, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(186.8, 186.88, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(172.75, 190.99, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(157.49, 193.06, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(141.76, 193.43, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(121.8, 192.98, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(102.04, 192.89, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(WaypointFollower(veh2, 9,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(86.51, 193.05, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(73.92, 193.22, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(58.03, 193.59, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(40.1, 193.87, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(23.89, 193.72, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(16.85, 193.57, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(StopVehicle(veh2, 1))
        citreon_sequence.add_child(WaypointFollower(veh2, 9,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(5.38, 193.46, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-6.4, 193.43, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-23.24, 192.37, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-39.11, 189.91, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-40.37, 180.88, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-34.95, 177.01, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-28.22, 173.86, 0), RoadOption.STRAIGHT)
                                                    ]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(StopVehicle(veh2, 1))
        citreon_sequence.add_child(Idle())

        # behavior of vehicle 3
        impala_sequence = py_trees.composites.Sequence("Impala_Sequence")
        others_node.add_child(impala_sequence)
        veh3 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh3")
        impala_sequence.add_child(InTriggerDistanceToLocation(veh2,
                                                               carla.Location(231.2, 22.1, 0), 4.0))
        impala_sequence.add_child(WaypointFollower(veh3, 15,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(207.79, 63.14, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(214.21, 63.11, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(222.95, 62.72, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(232.52, 60.65, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(239.6, 54.92, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(242.42, 45.18, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(241.48, 36.29, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(241.29, 26.5, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(241.45, 16.89, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(241.75, 7.85, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(242.04, -0.7, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(242.23, -9.84, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(242.59, -20.41, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(242.81, -29.77, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(243.04, -39.84, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        impala_sequence.add_child(ActorDestroy(veh3))

        # behavior of vehicle 4
        nissan_sequence = py_trees.composites.Sequence("Nissan_Sequence")
        others_node.add_child(nissan_sequence)
        veh4 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh4")
        nissan_sequence.add_child(InTriggerDistanceToLocation(veh2,
                                                               carla.Location(231.2, 22.1, 0), 4.0))
        nissan_sequence.add_child(WaypointFollower(veh4, 25,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(189.73, 62.59, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(199.47, 62.54, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(212.39, 62.73, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(223.83, 66.09, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(228.3, 73.28, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(229.01, 80.3, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(229.28, 90.39, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(229.08, 106.49, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(228.48, 128.51, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(227.79, 143.47, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(225.54, 155.59, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(218.54, 167.17, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(207.55, 176.21, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(194.21, 183.93, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(179.18, 189.27, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(162.85, 192.1, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(147.77, 192.95, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(130.72, 192.89, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(104.39, 192.83, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        # nissan_sequence.add_child(StopVehicle(veh4, 1))
        # nissan_sequence.add_child(Idle(2))
        nissan_sequence.add_child(WaypointFollower(veh4, 9,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(80.68, 192.89, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(67.37, 193.18, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(51.12, 193.44, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.74, 193.41, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(21.55, 193.49, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(12.38, 193.33, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        nissan_sequence.add_child(StopVehicle(veh4, 1))
        nissan_sequence.add_child(Idle(1))
        nissan_sequence.add_child(WaypointFollower(veh4, 9,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(9.13, 192.9, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(6.92, 191.54, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(4.69, 186.84, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(5.15, 181.5, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(6.08, 174.4, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(6.02, 165.41, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(5.78, 157.72, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(5.82, 148.53, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        nissan_sequence.add_child(StopVehicle(veh4, 1))
        nissan_sequence.add_child(Idle(8))
        nissan_sequence.add_child(WaypointFollower(veh4, 9,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(5.41, 145.47, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(6.66, 139.01, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(16.4, 133.42, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(27.37, 133.97, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.44, 134.06, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(39.92, 133.8, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(42.77, 132.99, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(44.56, 131.67, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(46.21, 128.57, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(46.17, 124.24, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(45.96, 121.0, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(45.97, 117.08, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(46.0, 115.58, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        # nissan_sequence.add_child(Idle(2))
        nissan_sequence.add_child(StopVehicle(veh4, 1))
        nissan_sequence.add_child(Idle())

        # behavior of vehicle 5
        # dodge_sequence = py_trees.composites.Sequence("Dodge_Sequence")
        # others_node.add_child(dodge_sequence)
        # veh5 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh5")
        # dodge_sequence.add_child(Idle(2))
        # dodge_sequence.add_child(WaypointFollower(veh5, 5,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(15.5, 201.36, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.83, 199.1, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(32.54, 191.5, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.48, 180.45, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.98, 170.98, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.98, 162.14, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.94, 158.18, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(35.01, 148.74, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.92, 136.26, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.92, 123.17, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.97, 116.7, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(35.07, 109.13, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # dodge_sequence.add_child(ActorDestroy(veh5))

        # # behavior of vehicle 6
        # mercedes_sequence = py_trees.composites.Sequence("Mercedes_Sequence")
        # others_node.add_child(mercedes_sequence)
        # veh6 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh6")
        # mercedes_sequence.add_child(Idle(45))
        # mercedes_sequence.add_child(WaypointFollower(veh6, 8,
        #                                             self.scenario_helper.get_waypoint_plan(locations=[
        #                                                 (carla.Location(33.9, -117.23, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(34.03, -120.12, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(34.5, -134.19, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(35.31, -151.9, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(35.84, -167.32, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(35.95, -178.15, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(35.21, -187.03, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(31.66, -195.22, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(22.61, -202.15, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(12.46, -203.72, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-0.41, -203.61, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-12.46, -203.74, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-25.12, -203.95, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-42.4, -204.0, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-59.91, -203.61, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-78.8, -204.15, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-101.42, -204.11, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-122.7, -204.28, 0), RoadOption.STRAIGHT)]),
        #                                             avoid_collision=True))
        # mercedes_sequence.add_child(ActorDestroy(veh6))

        # behavior of vehicle 7
        # toyota_sequence = py_trees.composites.Sequence("Toyota_Sequence")
        # others_node.add_child(toyota_sequence)
        # veh7 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh7")
        # toyota_sequence.add_child(Idle(3))
        # toyota_sequence.add_child(WaypointFollower(veh7, 18,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(-124.94, 187.8, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-145.97, 187.14, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-169.51, 182.52, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-191.38, 170.65, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-208.41, 154.57, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-222.71, 130.1, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-226.73, 101.46, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-226.46, 75.71, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-226.49, 45.65, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-226.13, 17.88, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-225.64, -10.75, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-225.75, -38.61, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-226.31, -69.35, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-226.37, -114.69, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-222.38, -127.0, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-215.34, -143.57, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-203.8, -159.37, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-185.88, -173.54, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-165.56, -182.59, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-141.71, -186.55, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-117.97, -186.65, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-94.07, -186.99, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-71.08, -187.05, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-51.28, -186.87, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-32.98, -186.75, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-18.66, -186.62, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-7.64, -186.49, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(-0.42, -186.42, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(10.27, -186.61, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(17.84, -186.78, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(32.26, -186.91, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(48.51, -186.9, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(67.94, -186.75, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(83.35, -186.83, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(95.44, -186.98, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # toyota_sequence.add_child(ActorDestroy(veh7))

        # behavior of pedestrians
        # behavior of pedestrian 1
        # ped1_sequence = py_trees.composites.Sequence("Pedestrian1_Sequence")
        # others_node.add_child(ped1_sequence)
        # ped1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped1")
        # ped1_sequence.add_child(Idle(14))
        # ped1_sequence.add_child(WaypointFollower(ped1, 3,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(38.5059, 131.652, 0), RoadOption.LANEFOLLOW),
        #                                             (carla.Location(38.6206, 103.154, 0), RoadOption.LANEFOLLOW)],
        #                                             lane_type=carla.LaneType.Sidewalk),
        #                                         avoid_collision=True))
        # ped1_sequence.add_child(ActorDestroy(ped1))

        # behavior of pedestrian 2
        # ped2_sequence = py_trees.composites.Sequence("Pedestrian2_Sequence")
        # others_node.add_child(ped2_sequence)
        # ped2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped2")
        # ped2_sequence.add_child(Idle(15))
        # ped2_sequence.add_child(WaypointFollower(ped2, 2,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(20.4881, 59.9498, 0), RoadOption.LANEFOLLOW)],
        #                                             lane_type=carla.LaneType.Sidewalk),
        #                                         avoid_collision=True))
        # ped2_sequence.add_child(ActorDestroy(ped2))

        tf1 = None
        tf2 = None
        tf3 = None
        for tf in self._world.get_traffic_lights_in_junction(498):
            if round(tf.get_transform().location.x, 2) == 167.35:
                tf1 = tf
            elif round(tf.get_transform().location.x, 2) == 163.68:
                tf2 = tf
            elif round(tf.get_transform().location.x, 2) == 140.56:
                tf3 = tf

        tf4 = None
        tf5 = None
        tf6 = None
        for tf in self._world.get_traffic_lights_in_junction(1352):
            if round(tf.get_transform().location.x, 2) == 248.73:
                tf4 = tf
            elif round(tf.get_transform().location.x, 2) == 249.4:
                tf5 = tf
            elif round(tf.get_transform().location.x, 2) == 224.51:
                tf6 = tf

        tf7 = None
        tf8 = None
        tf9 = None
        tf10 = None
        for tf in self._world.get_traffic_lights_in_junction(103):
            if round(tf.get_transform().location.x, 2) == 14.92:
                tf7 = tf
            elif round(tf.get_transform().location.x, 2) == 10.34:
                tf8 = tf
            elif round(tf.get_transform().location.x, 2) == -18.01:
                tf9 = tf
            elif round(tf.get_transform().location.x, 2) == -15.2:
                tf10 = tf

        #behavior of traffic_light 1
        tf1_sequence = py_trees.composites.Sequence("Traffic_Light1_Sequence")
        others_node.add_child(tf1_sequence)
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Red))
        tf1_sequence.add_child(Idle(10))
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Green))
        tf1_sequence.add_child(Idle(10))

        #behavior of traffic_light 2
        tf2_sequence = py_trees.composites.Sequence("Traffic_Lighmustang_sequence")
        others_node.add_child(tf2_sequence)
        tf2_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Green))
        tf2_sequence.add_child(Idle(10))
        tf2_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Red))
        tf2_sequence.add_child(Idle(10))

        #behavior of traffic_light 3
        tf3_sequence = py_trees.composites.Sequence("Traffic_Light3_Sequence")
        others_node.add_child(tf3_sequence)
        tf3_sequence.add_child(TrafficLightStateSetter(tf3, carla.TrafficLightState.Green))
        tf3_sequence.add_child(Idle(10))
        tf3_sequence.add_child(TrafficLightStateSetter(tf3, carla.TrafficLightState.Red))
        tf3_sequence.add_child(Idle(10))

        #behavior of traffic_light 4
        tf4_sequence = py_trees.composites.Sequence("Traffic_Light4_Sequence")
        others_node.add_child(tf4_sequence)
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Red))
        tf4_sequence.add_child(InTriggerDistanceToLocation(veh2,
                                                               carla.Location(231.2, 22.1, 0), 4.0))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Yellow))
        tf4_sequence.add_child(Idle(2))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Green))
        tf4_sequence.add_child(Idle(8))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Yellow))
        tf4_sequence.add_child(Idle(2))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Red))
        tf4_sequence.add_child(Idle())

        #behavior of traffic_light 5
        tf5_sequence = py_trees.composites.Sequence("Traffic_Light5_Sequence")
        others_node.add_child(tf5_sequence)
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Green))
        tf5_sequence.add_child(InTriggerDistanceToLocation(veh2,
                                                               carla.Location(231.2, 22.1, 0), 4.0))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Yellow))
        tf5_sequence.add_child(Idle(2))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Red))
        tf5_sequence.add_child(Idle(8))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Yellow))
        tf5_sequence.add_child(Idle(2))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Green))
        tf5_sequence.add_child(Idle())

        #behavior of traffic_light 6
        tf6_sequence = py_trees.composites.Sequence("Traffic_Light6_Sequence")
        others_node.add_child(tf6_sequence)
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Green))
        tf6_sequence.add_child(InTriggerDistanceToLocation(veh2,
                                                               carla.Location(231.2, 22.1, 0), 4.0))
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Yellow))
        tf6_sequence.add_child(Idle(2))
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Red))
        tf6_sequence.add_child(Idle(8))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Yellow))
        tf5_sequence.add_child(Idle(2))
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Green))
        tf6_sequence.add_child(Idle())
        
        #behavior of traffic_light 7
        tf7_sequence = py_trees.composites.Sequence("Traffic_Light7_Sequence")
        others_node.add_child(tf7_sequence)
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Green))
        tf7_sequence.add_child(InTriggerDistanceToLocation(veh4,
                                                               carla.Location(5.7, 169.9, 0), 4.0))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(Idle(2))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Red))
        tf7_sequence.add_child(Idle(8))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(Idle(2))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Green))
        tf7_sequence.add_child(Idle())
        
        #behavior of traffic_light 8
        tf8_sequence = py_trees.composites.Sequence("Traffic_Light8_Sequence")
        others_node.add_child(tf8_sequence)
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Red))
        tf8_sequence.add_child(InTriggerDistanceToLocation(veh4,
                                                               carla.Location(5.7, 169.9, 0), 4.0))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Yellow))
        tf8_sequence.add_child(Idle(2))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Green))
        tf8_sequence.add_child(Idle(8))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Yellow))
        tf8_sequence.add_child(Idle(2))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Red))
        tf8_sequence.add_child(Idle())
        
        #behavior of traffic_light 9
        tf9_sequence = py_trees.composites.Sequence("Traffic_Light9_Sequence")
        others_node.add_child(tf9_sequence)
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Green))
        tf9_sequence.add_child(InTriggerDistanceToLocation(veh4,
                                                               carla.Location(5.7, 169.9, 0), 4.0))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Yellow))
        tf9_sequence.add_child(Idle(2))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Red))
        tf9_sequence.add_child(Idle(8))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Yellow))
        tf9_sequence.add_child(Idle(2))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Green))
        tf9_sequence.add_child(Idle())

        #behavior of traffic_light 10
        tf10_sequence = py_trees.composites.Sequence("Traffic_Light10_Sequence")
        others_node.add_child(tf10_sequence)
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Red))
        tf10_sequence.add_child(InTriggerDistanceToLocation(veh4,
                                                               carla.Location(5.7, 169.9, 0), 4.0))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Yellow))
        tf10_sequence.add_child(Idle(2))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Green))
        tf10_sequence.add_child(Idle(8))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Yellow))
        tf10_sequence.add_child(Idle(2))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Red))
        tf10_sequence.add_child(Idle())

        return others_node


    def unfreezeVehicles(self):
        """
        This function unfreezes all vehicles in the simulation to allow moving of vehicles after freezing to
        maintain their position throughout scenario initialization by activating the physics simulation
        """
        for actor in self.other_actors:
            actor.set_simulate_physics(True)

        for ego in self.ego_vehicles:
            ego.set_simulate_physics(True)
