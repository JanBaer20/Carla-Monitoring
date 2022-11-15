import logging
from typing import List

import carla
import py_trees
from agents.navigation.local_planner import RoadOption
from py_trees.behaviour import Behaviour
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower, StopVehicle, Idle, ActorDestroy, LaneChange
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
    This class models the scenario "highway to town" where the ego vehicle drives from highway into town roads.

    Starting scenario with:
    $ python3 /opt/scenario_runner/scenario_runner.py --scenario VwScenarioHighwayTown --configFile="vw_scenario_highway_town.xml" --additionalScenario="vw_scenario_highway_town.py" --reloadWorld --debug
    """

    def __init__(self, world, ego_vehicles, config, terminate_on_failure=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
           Init function for the class VwScenarioHighwayTown

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
        t2_sequence = py_trees.composites.Sequence("VW_T2_Sequence")
        others_node.add_child(t2_sequence)
        veh1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh1")
        t2_sequence.add_child(WaypointFollower(veh1, 30,
                                            self.scenario_helper.get_waypoint_plan(locations=[
                                                (carla.Location(-141.5, 187.6, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-153.0, 186.6, 0), RoadOption.RIGHT),
                                                (carla.Location(-168.7, 183.1, 0), RoadOption.RIGHT),
                                                (carla.Location(-184.6, 175.9, 0), RoadOption.RIGHT),
                                                (carla.Location(-187.0, 173.9, 0), RoadOption.RIGHT),
                                                (carla.Location(-199.5, 164.7, 0), RoadOption.RIGHT),
                                                (carla.Location(-209.4, 153.6, 0), RoadOption.RIGHT),
                                                (carla.Location(-217.2, 141.2, 0), RoadOption.RIGHT),
                                                (carla.Location(-224.2, 122.2, 0), RoadOption.RIGHT),
                                                (carla.Location(-226.4, 105.8, 0), RoadOption.RIGHT),
                                                (carla.Location(-226.4, 61.4, 0), RoadOption.RIGHT),
                                                (carla.Location(-229.0, -18.8, 0), RoadOption.CHANGELANELEFT)]),
                                            avoid_collision=True))
        t2_sequence.add_child(ActorDestroy(veh1))

        # behavior of vehicle 2
        citreon_sequence = py_trees.composites.Sequence("Citreon_Sequence")
        others_node.add_child(citreon_sequence)
        veh2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh2")
        citreon_sequence.add_child(WaypointFollower(veh2, 7,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(-143.0, 190.8, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-204.5, 164.9, 0), RoadOption.RIGHT),
                                                        (carla.Location(-230.0, 98.5, 0), RoadOption.RIGHT)]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(ActorDestroy(veh2))

        # # behavior of vehicle 3
        # impala_sequence = py_trees.composites.Sequence("Impala_Sequence")
        # others_node.add_child(impala_sequence)
        # veh3 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh3")
        # impala_sequence.add_child(WaypointFollower(veh3, 4,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(44.2502, 188.366, 0), RoadOption.RIGHT),
        #                                             (carla.Location(40.0502, 187.996, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(36.4758, 185.718, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.212, 182.152, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.4205, 178.572, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.9716, 172.492, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.492, 155.751, 0), RoadOption.RIGHT),
        #                                             (carla.Location(35.2963, 150.994, 0), RoadOption.RIGHT),
        #                                             (carla.Location(36.3972, 145.872, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(37.7507, 144.868, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(38.7039, 144.47, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(40.7056, 144.224, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(42.6882, 144.186, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(69.6441, 146.004, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # impala_sequence.add_child(ActorDestroy(veh3))

        # # behavior of vehicle 4
        # nissan_sequence = py_trees.composites.Sequence("Nissan_Sequence")
        # others_node.add_child(nissan_sequence)
        # veh4 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh4")
        # nissan_sequence.add_child(WaypointFollower(veh4, 10,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(-67.3537, 194.28, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # nissan_sequence.add_child(ActorDestroy(veh4))

        # # behavior of vehicle 5
        # dodge_sequence = py_trees.composites.Sequence("Dodge_Sequence")
        # others_node.add_child(dodge_sequence)
        # veh5 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh5")
        # dodge_sequence.add_child(Idle(2))
        # dodge_sequence.add_child(WaypointFollower(veh5, 5,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(27.5, 170.456, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # dodge_sequence.add_child(StopVehicle(veh5, 25, ))
        # dodge_sequence.add_child(Idle(7))
        # dodge_sequence.add_child(WaypointFollower(veh5, 10,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(27.5, 177.456, 0), RoadOption.LEFT),
        #                                             (carla.Location(29.1435, 190.199, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(33.5917, 198.2, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(39.8191, 202.034, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(50.3676, 202.034, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(85.7891, 202.413, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # dodge_sequence.add_child(ActorDestroy(veh5))

        # # behavior of vehicle 6
        # mercedes_sequence = py_trees.composites.Sequence("Mercedes_Sequence")
        # others_node.add_child(mercedes_sequence)
        # veh6 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh6")
        # dodge_sequence.add_child(Idle(5))
        # mercedes_sequence.add_child(WaypointFollower(veh6, 8,
        #                                             self.scenario_helper.get_waypoint_plan(locations=[
        #                                                 (carla.Location(24.7165, 177.302, 0), RoadOption.LEFT),
        #                                                 (carla.Location(25.0342, 179.637, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(24.3776, 183.172, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(22.8525, 185.381, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(20.0141, 187.59, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(15.2482, 188.432, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-2.16317, 188.221, 0), RoadOption.STRAIGHT),
        #                                                 (carla.Location(-45.7129, 188.474, 0), RoadOption.STRAIGHT)]),
        #                                             avoid_collision=True))
        # mercedes_sequence.add_child(ActorDestroy(veh6))

        # # behavior of vehicle 7
        # toyota_sequence = py_trees.composites.Sequence("Toyota_Sequence")
        # others_node.add_child(toyota_sequence)
        # veh7 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh7")
        # toyota_sequence.add_child(Idle(6))
        # toyota_sequence.add_child(WaypointFollower(veh7, 7,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(65.8481, 142.453, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(42.8668, 141.598, 0), RoadOption.RIGHT),
        #                                             (carla.Location(38.305, 141.171, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(35.9811, 140.23, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.6039, 137.836, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.6596, 134.417, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(34.69, 130.655, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(35.1204, 122.61, 0), RoadOption.CHANGELANELEFT),
        #                                             (carla.Location(33.3129, 119.45, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(31.5914, 114.069, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(31.5914, 108.682, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # toyota_sequence.add_child(StopVehicle(veh7, 25, ))
        # toyota_sequence.add_child(Idle(4))
        # toyota_sequence.add_child(WaypointFollower(veh7, 8,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(31.3332, 54.82, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # toyota_sequence.add_child(ActorDestroy(veh7))

        # # behavior of vehicle 8
        # seat_sequence = py_trees.composites.Sequence("Seat_Leon_Sequence")
        # others_node.add_child(seat_sequence)
        # veh8 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh8")
        # seat_sequence.add_child(Idle(18))
        # seat_sequence.add_child(WaypointFollower(veh8, 7,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(16.9879, 95.117, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(18.9388, 94.9468, 0), RoadOption.RIGHT),
        #                                             (carla.Location(20.775, 95.345, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(23.5867, 96.8839, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.1032, 98.3659, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.7344, 99.8478, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.7344, 101.786, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.677, 103.781, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.4474, 109.936, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.5622, 121.963, 0), RoadOption.STRAIGHT),
        #                                             (carla.Location(24.5622, 172.591, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # seat_sequence.add_child(ActorDestroy(veh8))

        # # behavior of vehicle 9
        # police_sequence = py_trees.composites.Sequence("Police_Sequence")
        # others_node.add_child(police_sequence)
        # veh9 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh9")
        # police_sequence.add_child(Idle(22))
        # police_sequence.add_child(WaypointFollower(veh9, 10,
        #                                         self.scenario_helper.get_waypoint_plan(locations=[
        #                                             (carla.Location(-16.5802, 84.6866, 0), RoadOption.STRAIGHT)]),
        #                                         avoid_collision=True))
        # police_sequence.add_child(ActorDestroy(veh9))

        # # behavior of vehicle 10
        # mustang_sequence = py_trees.composites.Sequence("Mustang_Sequence")
        # others_node.add_child(mustang_sequence)
        # veh10 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh10")
        # mustang_sequence.add_child(Idle(23))
        # mustang_sequence.add_child(WaypointFollower(veh10, 9,
        #                                             self.scenario_helper.get_waypoint_plan(locations=[
        #                                                 (carla.Location(28.0625, 174.514, 0), RoadOption.STRAIGHT)]),
        #                                             avoid_collision=True))
        # mustang_sequence.add_child(ActorDestroy(veh10))

        # behavior of pedestrians
        # behavior of pedestrian 1
        ped1_sequence = py_trees.composites.Sequence("Pedestrian1_Sequence")
        others_node.add_child(ped1_sequence)
        ped1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped1")
        ped1_sequence.add_child(Idle(14))
        ped1_sequence.add_child(WaypointFollower(ped1, 3,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(38.5059, 131.652, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(38.6206, 103.154, 0), RoadOption.STRAIGHT)],
                                                    lane_type=carla.LaneType.Sidewalk),
                                                avoid_collision=True))
        ped1_sequence.add_child(ActorDestroy(ped1))

        # behavior of pedestrian 2
        ped2_sequence = py_trees.composites.Sequence("Pedestrian2_Sequence")
        others_node.add_child(ped2_sequence)
        ped2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped2")
        ped2_sequence.add_child(Idle(15))
        ped2_sequence.add_child(WaypointFollower(ped2, 2,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(20.4881, 59.9498, 0), RoadOption.STRAIGHT)],
                                                    lane_type=carla.LaneType.Sidewalk),
                                                avoid_collision=True))
        ped2_sequence.add_child(ActorDestroy(ped2))

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
