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
from carla_monitors.vehicle_on_route import VehicleOnRoute
# Initialization of logging
from carla_noise_generator.normal_noise_generator import NoiseTypeEnum

logger = logging.getLogger(__name__)
logging.basicConfig(format='%(module)s - %(levelname)s: %(message)s', level=logging.INFO)


class PruefungLandScenario(BasicScenario):
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
        self.scenario_name = "PruefungLandScenario"
        self.subtype = config.subtype

        # Call constructor of BasicScenario
        super(PruefungLandScenario, self).__init__(
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
        world.freeze_all_traffic_lights(True)

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
        
        route = [carla.Location(-197.81, -129.63, 0),
carla.Location(-197.96, -137.16, 0),
carla.Location(-198.11, -146.33, 0),
carla.Location(-198.18, -158.14, 0),
carla.Location(-198.44, -169.26, 0),
carla.Location(-198.55, -180.75, 0),
carla.Location(-198.5, -191.92, 0),
carla.Location(-198.47, -204.29, 0),
carla.Location(-198.34, -219.01, 0),
carla.Location(-196.9, -230.8, 0),
carla.Location(-190.8, -240.43, 0),
carla.Location(-180.93, -244.35, 0),
carla.Location(-169.59, -243.77, 0),
carla.Location(-157.0, -241.92, 0),
carla.Location(-145.02, -237.8, 0),
carla.Location(-134.38, -230.18, 0),
carla.Location(-126.47, -220.84, 0),
carla.Location(-118.54, -213.0, 0),
carla.Location(-108.39, -207.84, 0),
carla.Location(-98.55, -205.68, 0),
carla.Location(-85.96, -207.54, 0),
carla.Location(-75.25, -213.82, 0),
carla.Location(-67.14, -224.14, 0),
carla.Location(-61.13, -232.95, 0),
carla.Location(-51.51, -240.34, 0),
carla.Location(-40.26, -243.41, 0),
carla.Location(-29.94, -243.1, 0),
carla.Location(-20.17, -241.28, 0),
carla.Location(-9.0, -239.27, 0),
carla.Location(0.74, -239.06, 0),
carla.Location(10.32, -237.88, 0),
carla.Location(20.6, -233.89, 0),
carla.Location(28.82, -227.95, 0),
carla.Location(34.57, -220.47, 0),
carla.Location(37.97, -210.17, 0),
carla.Location(39.54, -199.03, 0),
carla.Location(40.69, -187.19, 0),
carla.Location(41.61, -175.6, 0),
carla.Location(43.25, -164.42, 0),
carla.Location(47.04, -154.62, 0),
carla.Location(53.03, -145.31, 0),
carla.Location(58.91, -135.74, 0),
carla.Location(62.31, -126.49, 0),
carla.Location(63.05, -115.63, 0),
carla.Location(61.28, -104.36, 0),
carla.Location(59.14, -92.04, 0),
carla.Location(57.4, -79.67, 0),
carla.Location(56.12, -67.69, 0),
carla.Location(57.46, -55.13, 0),
carla.Location(63.38, -44.91, 0),
carla.Location(70.1, -36.08, 0),
carla.Location(73.06, -27.82, 0),
carla.Location(72.63, -20.26, 0),
carla.Location(69.97, -11.81, 0),
carla.Location(65.09, -5.34, 0),
carla.Location(58.52, -3.61, 0),
carla.Location(51.48, -3.54, 0),
carla.Location(43.11, -3.85, 0),
carla.Location(35.33, -4.06, 0),
carla.Location(27.68, -4.67, 0),
carla.Location(19.72, -6.15, 0),
carla.Location(11.58, -6.49, 0),
carla.Location(4.38, -7.45, 0),
carla.Location(-0.61, -12.43, 0),
carla.Location(-1.54, -19.68, 0),
carla.Location(-1.66, -26.91, 0),
carla.Location(-1.48, -34.29, 0),
carla.Location(-1.39, -42.12, 0),
carla.Location(-1.11, -49.57, 0),
carla.Location(-1.1, -56.76, 0),
carla.Location(-3.87, -62.52, 0),
carla.Location(-10.05, -64.75, 0),
carla.Location(-16.04, -64.92, 0),
carla.Location(-22.52, -65.26, 0),
carla.Location(-29.55, -65.6, 0),
carla.Location(-36.9, -65.65, 0),
carla.Location(-43.82, -65.62, 0),
carla.Location(-50.52, -66.22, 0)]

        turningPoints = [(carla.Location(73.96, -19.27, 0), "Turn Right"),
(carla.Location(31.1, -4.44, 0), "Turn Right"),
(carla.Location(-1.69, -34.48, 0), "Turn Left")]
        
        # final location for scneario end
        loc = carla.Location(-55.8, -70.2, 0)
        dist = 5.0
        criteria = [ReachedRegionTest(self.ego_vehicles[0], loc.x - dist, loc.x + dist, loc.y - dist, loc.y + dist), VehicleOnRoute(self.ego_vehicles[0], self._world, route, turningPoints, False)]
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
        t2_sequence.add_child(WaypointFollower(veh1, 14,
                                            self.scenario_helper.get_waypoint_plan(locations=[
                                                (carla.Location(-180.94, -162.72, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-185.37, -162.84, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-189.23, -162.9, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-192.43, -162.92, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-194.38, -162.79, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-197.39, -161.88, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-200.05, -159.69, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.68, -156.23, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-202.02, -152.15, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.48, -147.72, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.11, -141.37, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.06, -137.58, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.16, -132.86, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.24, -128.28, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.28, -123.33, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.32, -117.92, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.33, -112.82, 0), RoadOption.STRAIGHT),
                                                (carla.Location(-201.3, -107.97, 0), RoadOption.STRAIGHT)]),
                                            avoid_collision=True))
        t2_sequence.add_child(ActorDestroy(veh1))

        # # behavior of vehicle 2
        citreon_sequence = py_trees.composites.Sequence("Citreon_Sequence")
        others_node.add_child(citreon_sequence)
        veh2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh2")
        citreon_sequence.add_child(WaypointFollower(veh2, 16,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(-198.26, -185.17, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-198.35, -191.54, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-198.45, -199.29, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-198.44, -206.84, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-198.41, -215.83, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-198.37, -224.06, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-197.36, -231.49, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-193.88, -238.63, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-186.75, -243.99, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-177.63, -245.19, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-169.51, -244.02, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-160.09, -242.67, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-151.31, -241.06, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-141.35, -236.65, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-134.55, -230.98, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-127.85, -222.69, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-121.33, -214.9, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-114.75, -209.79, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-107.07, -206.45, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-98.68, -204.93, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-88.89, -206.0, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-79.04, -210.52, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-72.78, -215.98, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-67.24, -223.57, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-62.67, -230.93, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-58.4, -235.76, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-51.06, -240.91, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-42.68, -243.46, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-34.13, -243.68, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-25.58, -242.34, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-20.02, -241.28, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(WaypointFollower(veh2, 15,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(-15.07, -240.34, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-9.78, -238.72, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-6.51, -236.16, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-4.51, -232.92, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-3.6, -228.55, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-3.69, -223.77, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-3.97, -218.72, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-3.99, -213.07, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-3.88, -207.17, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-3.84, -199.66, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(ActorDestroy(veh2))

        # # behavior of vehicle 3
        sprinter_sequence = py_trees.composites.Sequence("Sprinter_Sequence")
        others_node.add_child(sprinter_sequence)
        veh3 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh3")
        sprinter_sequence.add_child(InTriggerDistanceToLocation(veh2,
                                                               carla.Location(-26.1, -243.1, 0), 4.0))
        sprinter_sequence.add_child(WaypointFollower(veh3, 6,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(-0.35, -208.29, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(0.29, -214.12, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(0.31, -220.63, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(0.26, -227.86, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(0.6, -232.1, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(2.42, -235.09, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(5.72, -237.42, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(9.27, -238.34, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        sprinter_sequence.add_child(WaypointFollower(veh3, 12,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(12.33, -237.6, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(17.5, -235.28, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(24.2, -231.75, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(31.69, -226.15, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(35.9, -218.85, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(37.64, -212.55, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(38.63, -206.69, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(39.57, -196.62, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(40.41, -187.55, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(41.17, -178.12, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(42.32, -168.1, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(43.98, -161.32, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(46.89, -154.5, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(50.99, -147.37, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(55.31, -141.79, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(59.22, -136.23, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(62.14, -129.5, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(63.56, -122.91, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(63.65, -114.52, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(62.56, -107.38, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(59.76, -96.21, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(57.97, -83.51, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(56.59, -73.37, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(55.59, -64.82, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        sprinter_sequence.add_child(WaypointFollower(veh3, 5,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(56.01, -57.92, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(58.97, -49.25, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(65.08, -40.74, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(70.15, -35.31, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(73.54, -28.95, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(74.12, -24.14, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(73.48, -19.18, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(71.76, -14.23, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(69.86, -10.49, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        sprinter_sequence.add_child(StopVehicle(veh3, 1))
        sprinter_sequence.add_child(Idle(2))
        sprinter_sequence.add_child(WaypointFollower(veh3, 10,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(68.6, -7.16, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(67.68, -3.47, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(68.78, 1.38, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(73.17, 6.37, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(79.21, 12.74, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(82.05, 23.04, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(81.48, 35.71, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        sprinter_sequence.add_child(ActorDestroy(veh3))

        # # behavior of vehicle 4
        nissan_sequence = py_trees.composites.Sequence("Nissan_Sequence")
        others_node.add_child(nissan_sequence)
        veh4 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh4")
        nissan_sequence.add_child(InTriggerDistanceToLocation(veh3,
                                                               carla.Location(70.0, -36.0, 0), 4.0))
        nissan_sequence.add_child(WaypointFollower(veh4, 15,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(31.86, -0.29, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(38.25, -0.09, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(46.68, 0.07, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(58.53, 1.2, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(68.55, 4.36, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(76.2, 10.86, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(80.95, 20.38, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(81.64, 30.45, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(79.83, 43.67, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(74.66, 51.54, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(67.24, 56.82, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        nissan_sequence.add_child(ActorDestroy(veh4))
        

        # # behavior of vehicle 5
        dodge_sequence = py_trees.composites.Sequence("Dodge_Sequence")
        others_node.add_child(dodge_sequence)
        veh5 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh5")
        dodge_sequence.add_child(InTriggerDistanceToLocation(veh3,
                                                               carla.Location(70.0, -36.0, 0), 4.0))
        dodge_sequence.add_child(WaypointFollower(veh5, 8,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(85.76, 23.27, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(84.55, 18.18, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(80.33, 9.42, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(71.78, 1.3, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(59.0, -2.76, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(42.24, -3.48, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(27.87, -3.58, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(19.56, -3.54, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(15.37, -3.37, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        dodge_sequence.add_child(StopVehicle(veh5, 1))
        dodge_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(15.2, -6.8, 0), 4.0))
        dodge_sequence.add_child(Idle(6))
        dodge_sequence.add_child(WaypointFollower(veh5, 8,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(4.76, -3.22, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-0.86, -3.21, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-8.2, -3.17, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-15.84, -3.1, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-23.96, -2.9, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-31.24, -2.85, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-38.22, -3.29, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-43.23, -6.61, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-43.98, -11.82, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-43.99, -12.98, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        dodge_sequence.add_child(StopVehicle(veh5, 1))
        dodge_sequence.add_child(Idle())
        # dodge_sequence.add_child(ActorDestroy(veh5))

        # # behavior of vehicle 6
        mercedes_sequence = py_trees.composites.Sequence("Mercedes_Sequence")
        others_node.add_child(mercedes_sequence)
        veh6 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh6")
        mercedes_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(15.2, -6.8, 0), 4.0))
        mercedes_sequence.add_child(WaypointFollower(veh6, 10,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(-2.24, 21.34, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-2.08, 9.11, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-1.87, -8.07, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-1.77, -31.32, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-1.68, -45.94, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-1.65, -64.63, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-1.37, -78.18, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-1.2, -89.38, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-1.92, -97.59, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-3.82, -102.63, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-6.63, -106.03, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-9.74, -107.98, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-13.06, -108.93, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-17.75, -109.18, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-28.54, -108.76, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-41.78, -109.64, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-49.32, -110.22, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True))
        mercedes_sequence.add_child(ActorDestroy(veh6))

        # behavior of vehicle 7
        toyota_sequence = py_trees.composites.Sequence("Toyota_Sequence")
        others_node.add_child(toyota_sequence)
        veh7 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh7")
        toyota_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(30.3, -226.8, 2.0), 4.0))
        toyota_sequence.add_child(WaypointFollower(veh7, 12,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(76.02, -14.61, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(76.84, -17.88, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(77.3, -20.59, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(77.47, -23.63, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(77.2, -26.76, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(75.76, -31.84, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(73.56, -36.0, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(70.32, -40.02, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(66.66, -44.02, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(64.01, -47.6, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(62.11, -51.09, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(60.95, -53.97, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(59.81, -58.04, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(59.25, -62.19, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(59.28, -66.65, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(59.74, -71.14, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(60.46, -76.08, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(61.28, -81.04, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(62.13, -85.73, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(63.21, -91.53, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(64.36, -97.74, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(65.57, -103.8, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(66.68, -110.5, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(67.24, -116.37, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(66.99, -122.47, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(65.86, -128.35, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(63.6, -134.63, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(60.85, -139.65, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(57.13, -144.88, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(52.91, -150.51, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(48.39, -158.64, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(46.43, -165.19, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(45.35, -173.55, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(44.55, -182.03, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(43.91, -189.23, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(43.36, -195.91, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(42.81, -201.89, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(41.92, -208.98, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(40.13, -217.51, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(36.99, -225.35, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(31.77, -231.78, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(24.42, -236.41, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(16.29, -239.67, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(7.8, -241.82, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(0.78, -242.55, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-4.87, -243.06, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        toyota_sequence.add_child(ActorDestroy(veh7))

        # # behavior of vehicle 8
        seat_sequence = py_trees.composites.Sequence("Seat_Sequence")
        others_node.add_child(seat_sequence)
        veh8 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh8")
        seat_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(-174.9, -245.1, 0), 4.0))
        seat_sequence.add_child(WaypointFollower(veh8, 12,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(-13.44, -243.92, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-21.36, -245.22, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-32.51, -247.2, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-41.89, -247.38, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-45.89, -246.66, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-50.79, -244.98, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-55.76, -242.23, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-61.35, -237.59, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-65.32, -232.93, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-68.58, -227.89, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-72.5, -221.66, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-78.68, -215.39, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-84.16, -211.79, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-91.15, -209.0, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-97.42, -208.14, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-103.72, -208.85, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-110.74, -211.54, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-116.63, -215.5, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-120.18, -218.97, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-124.21, -224.27, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-128.72, -230.32, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-134.25, -235.64, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-140.58, -239.87, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-147.46, -242.93, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-155.39, -245.08, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-164.94, -246.86, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-173.6, -248.58, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-182.44, -249.03, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-190.38, -246.53, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-196.81, -241.19, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-200.33, -235.19, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-201.65, -229.0, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-201.96, -220.12, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-202.01, -209.37, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-201.7, -198.59, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True))
        seat_sequence.add_child(ActorDestroy(veh8))

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
        for tf in self._world.get_traffic_lights_in_junction(918):
            if round(tf.get_transform().location.x, 2) == 4.91:
                tf1 = tf
            elif round(tf.get_transform().location.x, 2) == 4.46:
                tf2 = tf
            elif round(tf.get_transform().location.x, 2) == 6.36:
                tf3 = tf

        tf4 = None
        tf5 = None
        tf6 = None
        for tf in self._world.get_traffic_lights_in_junction(466):
            if round(tf.get_transform().location.x, 2) == 55.77:
                tf4 = tf
            elif round(tf.get_transform().location.x, 2) == 68.93:
                tf5 = tf
            elif round(tf.get_transform().location.x, 2) == 62.72:
                tf6 = tf

        tf7 = None
        tf8 = None
        tf9 = None
        tf10 = None
        tf11 = None
        for tf in self._world.get_traffic_lights_in_junction(725):
            if round(tf.get_transform().location.x, 2) == -10.94:
                tf7 = tf
            elif round(tf.get_transform().location.x, 2) == 5.91:
                tf8 = tf
            elif round(tf.get_transform().location.x, 2) == 2.28:
                tf9 = tf
            elif round(tf.get_transform().location.x, 2) == -12.22:
                tf10 = tf
                tf11 = tf

        # tf11 = None
        # tf12 = None
        # tf13 = None
        # tf14 = None
        # for tf in self._world.get_traffic_lights_in_junction(238):
        #     if round(tf.get_transform().location.x, 2) == -67.82:
        #         tf11 = tf
        #     elif round(tf.get_transform().location.x, 2) == -96.89:
        #         tf12 = tf
        #     elif round(tf.get_transform().location.x, 2) == -93.98:
        #         tf13 = tf
        #     elif round(tf.get_transform().location.x, 2) == -66.79:
        #         tf14 = tf

        #behavior of traffic_light 1
        tf1_sequence = py_trees.composites.Sequence("Traffic_Light1_Sequence")
        others_node.add_child(tf1_sequence)
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Green))
        tf1_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Red))
        tf1_sequence.add_child(TrafficLightStateSetter(tf3, carla.TrafficLightState.Green))
        tf1_sequence.add_child(InTriggerDistanceToLocation(veh2,
                                                               carla.Location(-26.1, -243.1, 0), 4.0))
        tf1_sequence.add_child(Idle(1))
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Yellow))
        tf1_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Yellow))
        tf1_sequence.add_child(TrafficLightStateSetter(tf3, carla.TrafficLightState.Yellow))
        tf1_sequence.add_child(Idle(2))
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Red))
        tf1_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Green))
        tf1_sequence.add_child(TrafficLightStateSetter(tf3, carla.TrafficLightState.Red))
        tf1_sequence.add_child(Idle(10))
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Yellow))
        tf1_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Yellow))
        tf1_sequence.add_child(TrafficLightStateSetter(tf3, carla.TrafficLightState.Yellow))
        tf1_sequence.add_child(Idle(2))
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Green))
        tf1_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Red))
        tf1_sequence.add_child(TrafficLightStateSetter(tf3, carla.TrafficLightState.Green))

        #behavior of traffic_light 4
        tf4_sequence = py_trees.composites.Sequence("Traffic_Light4_Sequence")
        others_node.add_child(tf4_sequence)
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Red))
        tf4_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Green))
        tf4_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Green))
        tf4_sequence.add_child(InTriggerDistanceToLocation(veh3,
                                                               carla.Location(70.0, -36.2, 0), 4.0))
        tf4_sequence.add_child(Idle(5))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Yellow))
        tf4_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Yellow))
        tf4_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Yellow))
        tf4_sequence.add_child(Idle(2))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Green))
        tf4_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Red))
        tf4_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Red))
        tf4_sequence.add_child(Idle())
        
        # #behavior of traffic_light 7
        tf7_sequence = py_trees.composites.Sequence("Traffic_Light7_Sequence")
        others_node.add_child(tf7_sequence)
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Red))
        tf7_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Red))
        tf7_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Green))
        tf7_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Green))
        tf7_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Green))
        tf7_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(15.2, -6.8, 0), 4.0))
        tf7_sequence.add_child(Idle(5))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(Idle(2))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Green))
        tf7_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Green))
        tf7_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Red))
        tf7_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Red))
        tf7_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Red))
        tf7_sequence.add_child(Idle(8))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(Idle(2))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Red))
        tf7_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Green))
        tf7_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Red))
        tf7_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Green))
        tf7_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Green))
        tf7_sequence.add_child(Idle())
        
        # #behavior of traffic_light 7
        # tf8_sequence = py_trees.composites.Sequence("Traffic_Light8_Sequence")
        # others_node.add_child(tf8_sequence)
        # tf8_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Red))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf12, carla.TrafficLightState.Green))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf13, carla.TrafficLightState.Red))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf14, carla.TrafficLightState.Green))
        # tf8_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
        #                                                        carla.Location(-30.9, 131.6, 0), 4.0))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf12, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf13, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf14, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(Idle(2))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Green))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf12, carla.TrafficLightState.Red))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf13, carla.TrafficLightState.Green))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf14, carla.TrafficLightState.Red))
        # tf8_sequence.add_child(Idle(8))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf12, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf13, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf14, carla.TrafficLightState.Yellow))
        # tf8_sequence.add_child(Idle(2))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf11, carla.TrafficLightState.Red))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf12, carla.TrafficLightState.Green))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf13, carla.TrafficLightState.Red))
        # tf8_sequence.add_child(TrafficLightStateSetter(tf14, carla.TrafficLightState.Green))
        # tf8_sequence.add_child(Idle())

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
