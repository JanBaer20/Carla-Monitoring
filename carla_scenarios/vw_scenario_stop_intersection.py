import logging

import py_trees.composites
from py_trees.behaviour import Behaviour
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ReachedRegionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario

from carla_helpers import ScenarioHelper
from carla_monitors.monitor_ego_vehicle import MonitorEgoVehicle, NoiseData, NoiseTargetEnum
from carla_noise_generator.normal_noise_generator import NoiseTypeEnum

# Initialization of logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(module)s - %(levelname)s: %(message)s', level=logging.INFO)


class VwScenarioStopIntersection(BasicScenario):
    """
    This class models the scenario "Stop Intersection" driving multiple intersection missing lights.

    Starting scenario with:
    $ python3 /opt/scenario_runner/scenario_runner.py --scenario VwScenarioStopIntersection --configFile="vw_scenario_stop_intersection.xml" --additionalScenario="vw_scenario_stop_intersection.py" --reloadWorld --debug


    """

    def __init__(self, world, ego_vehicles, config, terminate_on_failure=False, debug_mode=False, criteria_enable=True,
                 timeout=90):
        """
             Init function for the class VwScenario1

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
        self.subtype = config.subtype
        self.scenario_name = "VwScenarioStopIntersection"

        # Call constructor of BasicScenario
        super(VwScenarioStopIntersection, self).__init__(
            self.scenario_name,
            ego_vehicles,
            config,
            world,
            terminate_on_failure=terminate_on_failure,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable)

        # set a new birds eye position for spectator
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(106.609, 10.2162, 70),
                                                carla.Rotation(pitch=-90, yaw=-90)))

    def _create_behavior(self):
        """
        This method setups the behavior tree that contains the behavior of all non-ego vehicles during the scenario.

        Returns:
            Behaviour: the root node of the behavior tree
        """
        root = py_trees.composites.Parallel("alle actors")
        root.add_child(self.get_ego_behavior())

        other_vehicle_parallel = self.get_others_behavior()
        root.add_child(other_vehicle_parallel)

        return root

    def get_ego_behavior(self) -> Behaviour:
        """
         The behavior of the ego vehicle

          Returns:
              Behaviour: The py_tree node for the behavior
         """
        ego_node = py_trees.composites.Parallel("ego_vehicle")
        # we use WaypointFolloer with plan of (Waypoint, Road Option)
        ego_sequence = py_trees.composites.Sequence("Ego Sequence")
        ego_node.add_child(ego_sequence)
        ego_sequence.add_child(WaypointFollower(self.ego_vehicles[0], 10, self.scenario_helper.get_waypoint_plan([
            (carla.Location(66.9066, 1.63716, 0), RoadOption.STRAIGHT),
            (carla.Location(76.7556, 1.71548, 0), RoadOption.STRAIGHT)]), avoid_collision=True))
        ego_sequence.add_child(StopVehicle(self.ego_vehicles[0], 20, ))
        ego_sequence.add_child(Idle(7))
        ego_sequence.add_child(WaypointFollower(self.ego_vehicles[0], 8, self.scenario_helper.get_waypoint_plan([
            (carla.Location(114.217, 1.45441, 0), RoadOption.STRAIGHT),
            (carla.Location(122.234, 1.37608, 0), RoadOption.STRAIGHT),
            (carla.Location(145.051, 2.700, 0), RoadOption.LEFT),
            (carla.Location(151.45, 2.414691, 0), RoadOption.STRAIGHT),
            (carla.Location(153.363, -1.22163, 0), RoadOption.STRAIGHT),
            (carla.Location(155.257, -4.81143, 0), RoadOption.STRAIGHT),
            (carla.Location(155.687, -9.126, 0), RoadOption.STRAIGHT),
            (carla.Location(156.651, -11.25816, 0), RoadOption.STRAIGHT),
            (carla.Location(157.000, -32.9816, 0), RoadOption.STRAIGHT)]), avoid_collision=True))
        return ego_node

    def _create_test_criteria(self):
        """
        This method should setup a list with all evaluation criteria and monitors for the scenario.

        Returns:
             List[Criterion]: List of criterions and monitors for this scenario
        """
        loc = carla.Location(157.000, -34.9816, 0)
        dist = 3.0
        noise = NoiseData([NoiseTypeEnum.ALL_NOISE], NoiseTargetEnum.EGO, 0.0, 0.1, 2)
        return [CollisionTest(self.ego_vehicles[0]),
                ReachedRegionTest(self.ego_vehicles[0], loc.x - dist, loc.x + dist, loc.y - dist, loc.y + dist),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=None, scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 0.1, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 0.2, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 0.3, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 0.4, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 0.5, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 1.0, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 1.5, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 2.0, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 2.5, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 3.0, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 3.5, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 4.0, 2),
                                  scenario_name=self.scenario_name),
                MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                  generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                  NoiseTargetEnum.EGO, 0.0, 5.0, 2),
                                  scenario_name=self.scenario_name)
                ]

    def get_others_behavior(self) -> Behaviour:
        """
         The behavior of the other actors

          Returns:
              Behaviour: The py_tree node for the behavior
         """
        # composit for other vehicles
        others_node = py_trees.composites.Parallel("AllOtheActors")

        # Veh1 : VW T2
        t2_sequence = py_trees.composites.Sequence("VW_T2_Sequence")
        others_node.add_child(t2_sequence)
        veh1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh1")
        t2_sequence.add_child(WaypointFollower(veh1, 5,
                                               self.scenario_helper.get_waypoint_plan(locations=[
                                                   (carla.Location(87.802, 5.24002, 0), RoadOption.RIGHT),
                                                   (carla.Location(91.0349, 5.55331, 0), RoadOption.STRAIGHT),
                                                   (carla.Location(93.5582, 6.80648, 0), RoadOption.STRAIGHT),
                                                   (carla.Location(96.0025, 10.8009, 0), RoadOption.STRAIGHT),
                                                   (carla.Location(95.9237, 13.5423, 0), RoadOption.STRAIGHT),
                                                   (carla.Location(95.6346, 44.1404, 0), RoadOption.STRAIGHT)]),
                                               avoid_collision=True))
        t2_sequence.add_child(ActorDestroy(veh1))

        # Veh 3: Cireoen
        citreon_sequence = py_trees.composites.Sequence("Citroen_Sequence")
        veh2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh2")
        citreon_sequence.add_child(Idle(5))
        citreon_sequence.add_child(WaypointFollower(veh2, 4,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(x=106.108, y=13.3637, z=0), RoadOption.RIGHT),
                                                        (carla.Location(x=106.544, y=7.12192, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=107.243, y=3.5242, z=0), RoadOption.STRAIGHT),
                                                        (carla.Location(x=109.861, y=1.57363, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=112.13, y=1.05348, z=0), RoadOption.STRAIGHT),
                                                        (carla.Location(x=115.446, y=1.01014, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=128.189, y=1.57363, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=142.109, y=1.40025, z=0), RoadOption.RIGHT),
                                                        (carla.Location(x=145.775, y=1.53029, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=148.699, y=2.22382, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=150.793, y=3.87097, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=151.972, y=6.42839, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=152.364, y=10.1561, z=0),
                                                         RoadOption.STRAIGHT),
                                                        (carla.Location(x=152.059, y=17.655, z=0), RoadOption.STRAIGHT),
                                                        (carla.Location(x=151.71, y=34.3432, z=0),
                                                         RoadOption.STRAIGHT)]),
                                                    avoid_collision=True))
        citreon_sequence.add_child(ActorDestroy(veh2))
        others_node.add_child(citreon_sequence)

        # Veh 3: Impala
        impala_sequence = py_trees.composites.Sequence("Impala_Sequence")
        others_node.add_child(impala_sequence)
        veh3 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh3")
        impala_sequence.add_child(WaypointFollower(veh3, 6,
                                                   self.scenario_helper.get_waypoint_plan(locations=[
                                                       (carla.Location(99.8622, -12.8558, 0), RoadOption.RIGHT),
                                                       (carla.Location(99.8213, -10.0925, 0), RoadOption.RIGHT),
                                                       (carla.Location(94.7893, -5.94757, 0), RoadOption.STRAIGHT),
                                                       (carla.Location(92.171, -4.60656, 0), RoadOption.STRAIGHT),
                                                       (carla.Location(89.0209, -4.72847, 0), RoadOption.STRAIGHT),
                                                       (carla.Location(66.1927, -5.13483, 0), RoadOption.STRAIGHT),
                                                       (carla.Location(45.3602, -5.06372, 0), RoadOption.STRAIGHT)]),
                                                   avoid_collision=True))
        impala_sequence.add_child(ActorDestroy(veh3))

        # Veh 4: BMW
        bmw_sequence = py_trees.composites.Sequence("BMW_Sequence")
        veh4 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh4")
        bmw_sequence.add_child(Idle(20))
        bmw_sequence.add_child(WaypointFollower(veh4, 6,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(151.942, -12.875, 0), RoadOption.RIGHT),
                                                    (carla.Location(151.913, -9.20507, 0), RoadOption.RIGHT),
                                                    (carla.Location(150.284, -5.18834, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(147.317, -2.44309, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(142.778, - 1.51838, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(128.145, - 1.51838, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(71.5574, -1.38473, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=False))
        bmw_sequence.add_child(ActorDestroy(veh4))
        others_node.add_child(bmw_sequence)

        # Veh 5: Dodge
        dodge_sequence = py_trees.composites.Sequence("Dodge_Sequence")
        veh5 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh5")
        dodge_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                             carla.Location(140.729, 1.93124, 0), 1.0))
        dodge_sequence.add_child(WaypointFollower(veh5, 8,
                                                  self.scenario_helper.get_waypoint_plan(locations=[
                                                      (carla.Location(155.408, 12.2693, 0), RoadOption.STRAIGHT),
                                                      (carla.Location(155.114, 00.126962, 0), RoadOption.STRAIGHT),
                                                      (carla.Location(155.506, -13.72214, 0), RoadOption.STRAIGHT),
                                                      (carla.Location(155.015, -44.346, 0), RoadOption.STRAIGHT)]),
                                                  avoid_collision=True))
        dodge_sequence.add_child(ActorDestroy(veh5))
        others_node.add_child(dodge_sequence)

        # Veh 6: Sprinter
        sprinter_sequence = py_trees.composites.Sequence("Sprinter_Sequence")
        veh6 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh6")
        sprinter_sequence.add_child(Idle(5))
        sprinter_sequence.add_child(WaypointFollower(veh6, 9,
                                                     self.scenario_helper.get_waypoint_plan(locations=[
                                                         (carla.Location(103.615, -44.0047, 0), RoadOption.STRAIGHT)]),
                                                     avoid_collision=False))
        sprinter_sequence.add_child(ActorDestroy(veh6))
        others_node.add_child(sprinter_sequence)

        # Behavior of pedestrians
        ped1_sequence = py_trees.composites.Sequence("Pedestrian1_Sequence")

        # Pedestrian  1
        others_node.add_child(ped1_sequence)
        ped1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped1")
        ped1_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                            carla.Location(114.129, 1.5, 0), 1.0))
        ped1_sequence.add_child(WaypointFollower(ped1, 3,
                                                 self.scenario_helper.get_waypoint_plan(locations=[
                                                     (carla.Location(113.342, -9.49788, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(112.777, -10.8145, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(112.703, -17.9585, 0), RoadOption.STRAIGHT)],
                                                     lane_type=carla.LaneType.Sidewalk),
                                                 avoid_collision=True))

        # Pedestrian  2
        ped2_sequence = py_trees.composites.Sequence("Pedestrian2_Sequence")
        others_node.add_child(ped2_sequence)
        ped2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped2")
        ped2_sequence.add_child(Idle(7))
        ped2_sequence.add_child(WaypointFollower(ped2, 3,
                                                 self.scenario_helper.get_waypoint_plan(locations=[
                                                     (carla.Location(88.2599, -9.3929, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(89.089, -9.21952, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(90.6164, -9.86971, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(92.0564, -11.2134, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(110.515, -11.3868, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(110.603, -38.6948, 0), RoadOption.STRAIGHT)],
                                                     lane_type=carla.LaneType.Sidewalk),
                                                 avoid_collision=True))

        # Pedestrian  3
        ped3_sequence = py_trees.composites.Sequence("Pedestrian3_Sequence")
        others_node.add_child(ped3_sequence)
        ped3 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped3")
        ped3_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                            carla.Location(149.376, 0.328115, 0), 2.0))
        ped3_sequence.add_child(WaypointFollower(ped3, 3,
                                                 self.scenario_helper.get_waypoint_plan(locations=[
                                                     (carla.Location(144.516, -6.14534, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(143.706, 8.30107, 0), RoadOption.STRAIGHT),
                                                     (carla.Location(144, 40.4123, 0), RoadOption.STRAIGHT)],
                                                     lane_type=carla.LaneType.Sidewalk),
                                                 avoid_collision=True))

        return others_node
