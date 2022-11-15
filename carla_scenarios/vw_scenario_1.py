import logging
from typing import List, Tuple

import py_trees.composites
from py_trees import common
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ReachedRegionTest, \
    OutsideRouteLanesTest, Criterion
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation, \
    WaitEndIntersection, InTriggerDistanceToVehicle
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import *

from carla_helpers import ScenarioHelper
from carla_monitors.monitor_ego_vehicle import MonitorEgoVehicle, NoiseData, NoiseTargetEnum
from carla_noise_generator.normal_noise_generator import NoiseTypeEnum

# Initialization of logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(module)s - %(levelname)s: %(message)s', level=logging.INFO)


class VwScenario1(BasicScenario):
    """
    This class models the scenario "busy road" driving a corridor on Carla's map 'Town04' with heavy traffic along the ego
    vehicle's route.

    Starting scenario with:
    $ python3 /opt/scenario_runner/scenario_runner.py --scenario VwScenario1 --configFile="vw_scenario_1.xml" --additionalScenario="vw_scenario_1.py" --reloadWorld --debug


    """

    def __init__(self, world, ego_vehicles, config, terminate_on_failure=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
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

        # Apparently the class needs this attribute to be loaded by the ScenarioLoader
        self._ai_controllers = []
        self.timeout = timeout
        self._debug_mode = debug_mode
        self._map = world.get_map()
        self._world = world
        self.scenario_helper = ScenarioHelper(self._map)
        self.scenario_name = "VwScenarioStopIntersection"
        self.subtype = config.subtype

        # Call constructor of BasicScenario
        super(VwScenario1, self).__init__(
            self.scenario_name,
            ego_vehicles,
            config,
            world,
            debug_mode=debug_mode,
            terminate_on_failure=terminate_on_failure,
            criteria_enable=criteria_enable)

        # set a new position for spectator for birds eye view
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(130.435, -196.611, 110),
                                                carla.Rotation(pitch=-90, yaw=-90)))

    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        if config.other_actors:
            # requesting the creation of actors from the config
            new_actors = CarlaDataProvider.request_new_actors(config.other_actors)
            if not new_actors:
                raise Exception("Error: Unable to add actors")

            # saving the created actors for later destruction
            for new_actor in new_actors:
                self.other_actors.append(new_actor)

    def _create_behavior(self) -> py_trees.composites:
        """
        This method setups the behavior tree that contains the behavior of all non-ego vehicles during the scenario.
        The behavior tree should use py_trees and the atomic behaviors defined in atomic_scenario_behavior.py

        Returns:
            object: the root node of the behavior tree
        """
        root = py_trees.composites.Parallel("alle actors")
        # creating behavior for ego vehicle
        ego_node = py_trees.composites.Parallel("ego_vehicle")
        root.add_child(ego_node)
        # we use WaypointFolloer with plan of (Waypoint, Road Option)
        ego_sequence = py_trees.composites.Sequence("Ego Sequence")
        ego_node.add_child(ego_sequence)
        ego_sequence.add_child(WaypointFollower(self.ego_vehicles[0], 10, self.get_waypoint_ego_plan1(),
                                                avoid_collision=True))
        ego_sequence.add_child(WaypointFollower(self.ego_vehicles[0], 10, self.get_waypoint_ego_plan2(),
                                                avoid_collision=True))
        ego_reach_final_location = InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(203.631, -260.637, 0), 2.0)
        ego_node.add_child(ego_reach_final_location)  # ende des
        ego_light_manipulator = TrafficLightManipulator(self.ego_vehicles[0], self.subtype, debug=True)
        ego_node.add_child(ego_light_manipulator)

        # creating cmposite node for all other non-ego acteurs
        other_vehicle_parallel = py_trees.composites.Parallel("AllOtherVehicles")
        root.add_child(other_vehicle_parallel)

        # behavior of vehicle 1
        veh1_wp_follower = self.get_waypoint_follower_for_actor('veh1', 15,
                                                                self.get_waypoint_veh1_plan())
        other_vehicle_parallel.add_child(veh1_wp_follower)

        # behavior of vehicle 2
        veh2_wp_follower = self.get_waypoint_follower_for_actor('veh2', 15,
                                                                self.get_waypoint_veh2_plan())
        other_vehicle_parallel.add_child(veh2_wp_follower)

        # behavior of vehicle 3
        veh3_seq = py_trees.composites.Sequence('Impala Sequence')
        other_vehicle_parallel.add_child(veh3_seq)
        veh3_seq.add_child(
            self.get_waypoint_follower_for_actor('veh3', 15, self.get_waypoint_veh3_plan_start()))
        veh3_seq.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh3'), 5.0))
        veh3_seq.add_child(WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh6')))
        veh3_seq.add_child(
            self.get_waypoint_follower_for_actor('veh3', 15, self.get_waypoint_veh3_plan1()))
        veh3_seq.add_child(
            self.get_waypoint_follower_for_actor('veh3', 5, self.get_waypoint_veh3_plan2()))

        # behavior of vehicle 4
        veh4_seq = py_trees.composites.Sequence('Fire Truck Sequence')
        other_vehicle_parallel.add_child(veh4_seq)
        veh4_seq.add_child(
            self.get_waypoint_follower_for_actor('veh4', 5, self.get_waypoint_veh4_plan1()))
        veh4_seq.add_child(
            self.get_waypoint_follower_for_actor('veh4', 7, self.get_waypoint_veh4_plan2()))
        veh4_seq.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh4'), 20.0))
        veh4_seq.add_child(Idle())

        # behavior of vehicle 5
        veh5_sequence = py_trees.composites.Sequence('Dodge Charger Sequence')
        other_vehicle_parallel.add_child(veh5_sequence)
        veh5_sequence.add_child(
            InTriggerDistanceToVehicle(self.ego_vehicles[0],
                                       self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh5'), 60.0))
        veh5_sequence.add_child(
            self.get_waypoint_follower_for_actor('veh5', 10, self.get_waypoint_veh5_plan()))

        # behavior of vehicle 6
        veh6_seq = py_trees.composites.Sequence('Coupe Sequence')
        other_vehicle_parallel.add_child(veh6_seq)
        veh6_seq.add_child(
            self.get_waypoint_follower_for_actor('veh6', 15, self.get_waypoint_veh6_plan()))

        # behavior of vehicle 7
        veh7_seq = py_trees.composites.Sequence('Toyota Prius Sequence')
        other_vehicle_parallel.add_child(veh7_seq)
        veh7_seq.add_child(
            self.get_waypoint_follower_for_actor('veh7', 15, self.get_waypoint_veh7_plan_start()))
        veh7_seq.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh7'), 5.0))
        veh7_seq.add_child(WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh5')))
        veh7_seq.add_child(
            self.get_waypoint_follower_for_actor('veh7', 15, self.get_waypoint_veh7_plan()))

        # behavior of vehicle 8
        veh8_seq = py_trees.composites.Sequence('Coupe Sequence')
        other_vehicle_parallel.add_child(veh8_seq)
        veh8_seq.add_child(
            self.get_waypoint_follower_for_actor('veh8', 15, self.get_waypoint_veh8_plan_start()))
        veh8_seq.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh8'), 5.0))
        veh8_seq.add_child(
            WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh12'), debug=True))
        veh8_seq.add_child(
            self.get_waypoint_follower_for_actor('veh8', 15, self.get_waypoint_veh8_plan()))

        # behavior of vehicle 9
        veh9_seq = py_trees.composites.Sequence('Leon Sequence')
        other_vehicle_parallel.add_child(veh9_seq)
        veh9_seq.add_child(
            self.get_waypoint_follower_for_actor('veh9', 20, self.get_waypoint_veh9_plan_to_start()))
        veh9_seq.add_child(WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh4')))
        veh9_seq.add_child(
            self.get_waypoint_follower_for_actor('veh9', 20, self.get_waypoint_veh9_plan()))

        # behavior of vehicle 10
        veh10_seq = py_trees.composites.Sequence('VW T2 Sequence')
        other_vehicle_parallel.add_child(veh10_seq)
        veh10_seq.add_child(WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh4')))
        veh10_seq.add_child(
            self.get_waypoint_follower_for_actor('veh10', 15, self.get_waypoint_veh10_plan()))

        # behavior of vehicle 11
        veh11_seq = py_trees.composites.Sequence('Mini Sequence')
        other_vehicle_parallel.add_child(veh11_seq)
        veh11_seq.add_child(
            self.get_waypoint_follower_for_actor('veh11', 10, self.get_waypoint_veh11_plan_to_start()))
        veh11_seq.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh11'), 5.0))
        veh11_seq.add_child(
            WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh12')))
        veh11_seq.add_child(
            self.get_waypoint_follower_for_actor('veh11', 10, self.get_waypoint_veh11_plan_()))

        # behavior of vehicle 12
        veh12_seq = py_trees.composites.Sequence('Nissan patrol Sequence')
        other_vehicle_parallel.add_child(veh12_seq)
        veh12_seq.add_child(
            self.get_waypoint_follower_for_actor('veh12', 10, self.get_waypoint_veh12_plan_to_start()))
        veh12_seq.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh12'), 5.0))
        veh12_seq.add_child(WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh9')))
        veh12_seq.add_child(
            self.get_waypoint_follower_for_actor('veh12', 10, self.get_waypoint_veh12_plan()))

        # behavior of vehicle 13
        veh13_sequence = py_trees.composites.Sequence('Ford Mustang Sequence')
        other_vehicle_parallel.add_child(veh13_sequence)
        veh13_sequence.add_child(
            self.get_waypoint_follower_for_actor('veh13', 14, self.get_waypoint_veh13_plan_1()))
        veh13_sequence.add_child(
            StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh13'), 5.0))
        veh13_sequence.add_child(WaitEndIntersection(self.ego_vehicles[0]))
        veh13_sequence.add_child(
            self.get_waypoint_follower_for_actor('veh13', 14, self.get_waypoint_veh13_plan_2()))

        # behavior of vehicle 14
        veh14_sequence = py_trees.composites.Sequence('Transporter Sequence')
        other_vehicle_parallel.add_child(veh14_sequence)
        veh14_sequence.add_child(InTriggerDistanceToVehicle(
            self.ego_vehicles[0], self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh14'), 30.0))
        veh14_sequence.add_child(
            self.get_waypoint_follower_for_actor('veh14', 5, self.get_waypoint_veh14_plan()))

        # Creating sinks to destroy actors which are out of scope for the ego vehicle
        sinks_parallel = py_trees.composites.Parallel('vehicle_sinks')
        root.add_child(sinks_parallel)
        sinks_parallel.add_child(ActorSink(carla.Location(14.5243, -217.95, 0), 3.0))
        # sinks_parallel.add_child(ActorSink(carla.Location(206.214, -360.893, 0), 4.0))
        # sinks_parallel.add_child(ActorSink(carla.Location(329.18, -168.728, 0), 4.0))
        # sinks_parallel.add_child(ActorSink(carla.Location(275.732, -120, 0), 4.0))

        # COmposite Node for the walkers - pedestrians
        walkers_parallel = py_trees.composites.Parallel('AllWalkers')
        root.add_child(walkers_parallel)

        # Behavior of Pedestrian 1
        seq_ped1 = py_trees.composites.Sequence("Walker Ped1")
        walkers_parallel.add_child(seq_ped1)
        seq_ped1.add_child(self.get_waypoint_follower_for_actor('ped1', 2,
                                                                self.scenario_helper.get_waypoint_plan(
                                                                    [(carla.Location(122.732,
                                                                                     -180.797, 0),
                                                                      RoadOption.LANEFOLLOW)]),
                                                                False))
        seq_ped1.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped1'), 4.0))
        seq_ped1.add_child(WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh5')))
        seq_ped1.add_child(self.get_waypoint_follower_for_actor('ped1', 2,
                                                                self.scenario_helper.get_waypoint_plan(
                                                                    [(carla.Location(137.53, -178.374,
                                                                                     0),
                                                                      RoadOption.STRAIGHT)],
                                                                    lane_type=carla.LaneType.Sidewalk),
                                                                False))
        seq_ped1.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped1'), 2.0))

        # Behavior of Pedestrian 2
        seq_ped2 = py_trees.composites.Sequence("Walker Ped2")
        walkers_parallel.add_child(seq_ped2)
        seq_ped2.add_child(self.get_waypoint_follower_for_actor('ped2', 1,
                                                                self.scenario_helper.get_waypoint_plan(
                                                                    [(carla.Location(70.6176, -180.702,
                                                                                     0),
                                                                      RoadOption.STRAIGHT)],
                                                                    lane_type=carla.LaneType.Sidewalk),
                                                                False))
        seq_ped2.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped2'), 2.0))

        # Behavior of Pedestrian 5
        seq_ped5 = py_trees.composites.Sequence("Walker Ped5")
        walkers_parallel.add_child(seq_ped5)
        seq_ped5.add_child(WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh5')))
        parallel_ped5 = py_trees.composites.Parallel("IDLE Ped5", policy=common.ParallelPolicy.SUCCESS_ON_ONE)
        seq_ped5.add_child(parallel_ped5)
        parallel_ped5.add_child(Idle())
        parallel_ped5.add_child(
            WaitEndIntersection(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'veh5')))
        seq_ped5.add_child(self.get_waypoint_follower_for_actor('ped5', 5,
                                                                self.scenario_helper.get_waypoint_plan(
                                                                    [(
                                                                        carla.Location(209.0, -165.5, 0.2),
                                                                        RoadOption.STRAIGHT)],
                                                                    lane_type=carla.LaneType.Sidewalk),
                                                                False))
        seq_ped5.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped5'), 2.0))

        # Behavior of Pedestrian 8
        seq_ped8 = py_trees.composites.Sequence("Walker Ped8")
        walkers_parallel.add_child(seq_ped8)
        seq_ped8.add_child(InTriggerDistanceToVehicle(
            self.ego_vehicles[0], self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped8'), 50.0))
        seq_ped8.add_child(self.get_waypoint_follower_for_actor('ped8', 1,
                                                                self.scenario_helper.get_waypoint_plan(
                                                                    [(
                                                                        carla.Location(206.5, -206.0, 0.0),
                                                                        RoadOption.STRAIGHT)],
                                                                    lane_type=carla.LaneType.Sidewalk),
                                                                True))
        seq_ped8.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped8'), 2.0))

        # Behavior of Pedestrian 11
        seq_ped11 = py_trees.composites.Sequence("Walker Ped11")
        walkers_parallel.add_child(seq_ped11)
        seq_ped11.add_child(InTriggerDistanceToVehicle(
            self.ego_vehicles[0], self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped11'), 40.0))
        seq_ped11.add_child(self.get_waypoint_follower_for_actor('ped11', 1,
                                                                 self.scenario_helper.get_waypoint_plan(
                                                                     [(carla.Location(210.5, -239.0,
                                                                                      0.0),
                                                                       RoadOption.STRAIGHT)],
                                                                     lane_type=carla.LaneType.Sidewalk),
                                                                 False))
        seq_ped11.add_child(StopVehicle(self.scenario_helper.get_actor_by_role_name(self.other_actors, 'ped11'), 2.0))

        return root

    def get_ego_locations(self):
        """
        Defines the Waypoint for the ego vehicle
        Returns:
            List of carla.locations and Raodoptions
        """
        return [(carla.Location(42.2805, -170, 0), RoadOption.STRAIGHT),
                (carla.Location(79.4641, -170, 0), RoadOption.LANEFOLLOW),
                (carla.Location(114.197, -170, 0), RoadOption.STRAIGHT),
                (carla.Location(147.506, -170, 0), RoadOption.LANEFOLLOW),
                (carla.Location(195.6, -169, 0), RoadOption.LEFT),
                (carla.Location(200.956, - 170.681, 0), RoadOption.LANEFOLLOW),
                (carla.Location(203.018, -174.809, 0), RoadOption.LANEFOLLOW),
                (carla.Location(203.318, -176.809, 0), RoadOption.LANEFOLLOW),
                (carla.Location(203.776, -191.24, 0), RoadOption.LANEFOLLOW),
                (carla.Location(203.631, -230.637, 0), RoadOption.LANEFOLLOW),
                (carla.Location(203.631, -262.637, 0), RoadOption.LANEFOLLOW)]

    def get_waypoint_ego_plan1(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
        The first part of the ego route
        Returns:
             List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
        """
        # First three waypoint
        locations = self.get_ego_locations()[:4]
        locations.append((carla.Location(190.5, -168, 0), RoadOption.VOID))
        return self.scenario_helper.get_waypoint_plan(locations)

    def get_waypoint_ego_plan2(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
        The second part of the ego route
        Returns:
             List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
        """
        # From fourth waypoint till route end
        return self.scenario_helper.get_waypoint_plan(self.get_ego_locations()[4:])

    def get_waypoint_veh1_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
        The waypoints for the behavior of  vehicle 1
        Returns:
             List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
        """
        wp_locs = [(carla.Location(58.7025, -177.757, 0), RoadOption.RIGHT),
                   (carla.Location(52.6518, -174.121, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(21.054, -174.121, 0), RoadOption.RIGHT),
                   (carla.Location(14.7046, -181.541, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(14.6299, -220.347, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh2_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
                The waypoints for the behavior of  vehicle 2
                Returns:
                     List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
                """
        wp_locs = [(carla.Location(69.6086, -174.04, 0), RoadOption.STRAIGHT),
                   (carla.Location(51.5313, -173.75, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(18, -174.344, 0), RoadOption.RIGHT),
                   (carla.Location(14.7793, -180.428, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(14.0323, -220.496, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh3_plan_start(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
        The waypoints for the start behavior of  vehicle 3
        Returns:
             List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
        """
        wp_locs = [(carla.Location(116.301, -169.743, 0), RoadOption.VOID)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh3_plan1(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
                The waypoints for the middle route of  vehicle 3
                Returns:
                     List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
                """
        wp_locs = [(carla.Location(121.301, -169.743, 0), RoadOption.LEFT),
                   (carla.Location(131.31, -178.202, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(132.842, -199.785, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh3_plan2(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
                The waypoints for the final route part of  vehicle 4
                Returns:
                     List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
                """
        wp_locs = [(carla.Location(135.615, -206.379, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(140.722, -217.231, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(144.324, -223.3, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(154.706, -232.943, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(173.006, -242.912, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(194.73, -245.204, 0), RoadOption.RIGHT),
                   (carla.Location(199.246, -244, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(199.246, -237.68, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(198, -178.61, 0), RoadOption.STRAIGHT),
                   (carla.Location(197.606, -162.249, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(198.38, -156.112, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(200.494, -145.72, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(209.621, -129.69, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(219.304, -121.73, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(232.826, -118.745, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(239.4, -118.954, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(247.916, -118.645, 0), RoadOption.STRAIGHT),
                   (carla.Location(265.697, -118.983, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh4_plan1(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
                The waypoints for the beginning of route for vehicle 4
                Returns:
                     List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
                """
        wp_locs = [(carla.Location(202.221, -164.4, 0), RoadOption.LEFT),
                   (carla.Location(197.925, -171.832, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(194.329, -173.038, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(189.332, -173.224, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(142.533, -173.382, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(136.501, -173.499, 0), RoadOption.RIGHT),
                   (carla.Location(131.769, -176.598, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh4_plan2(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
                The waypoints for the end of route for vehicle 4
                Returns:
                     List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
                """
        wp_locs = [(carla.Location(131.31, -178.202, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(132.842, -199.785, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(135.615, -206.379, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(140.722, -217.231, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(144.324, -223.3, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(154.706, -232.943, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(173.006, -242.912, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(177.73, -243.204, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(182.73, -243.04, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh5_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the behavior of  vehicle 5
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(126.891, -180.546, 0), RoadOption.LEFT),
                   (carla.Location(127.64, -171.126, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(135.923, -168.895, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(140.898, -168.608, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(193.253, -168.608, 0), RoadOption.STRAIGHT),
                   (carla.Location(209.868, -168.416, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(248.12, -169.032, 0), RoadOption.STRAIGHT),
                   (carla.Location(268.406, -169.224, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh6_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the behavior of  vehicle 6
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(137.227, -173.35, 0), RoadOption.STRAIGHT),
                   (carla.Location(120.033, -173.35, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(69.6086, -174.04, 0), RoadOption.STRAIGHT),
                   (carla.Location(51.5313, -173.75, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(18, -174.344, 0), RoadOption.RIGHT),
                   (carla.Location(14.7793, -180.428, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(14.0323, -220.496, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh7_plan_start(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the route start  of  vehicle 7
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(140.227, -173.35, 0), RoadOption.VOID)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh7_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the remaining route of  vehicle 7
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(137.227, -173.35, 0), RoadOption.STRAIGHT),
                   (carla.Location(120.033, -173.35, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(69.6086, -174.04, 0), RoadOption.STRAIGHT),
                   (carla.Location(51.5313, -173.75, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(18, -174.344, 0), RoadOption.RIGHT),
                   (carla.Location(14.7793, -180.428, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(14.0323, -220.496, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh8_plan_start(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the route start of  vehicle 8
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(190.316, -169.416, 0), RoadOption.VOID)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh8_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the remaining route of  vehicle 1
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(193.316, -169.416, 0), RoadOption.RIGHT),
                   (carla.Location(198.362, -164.761, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(199.246, -150.7175, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(205.364, -135.187, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(211.346, -127.355, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(218.415, -122.494, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(234.389, -118.848, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(247.916, -118.645, 0), RoadOption.STRAIGHT),
                   (carla.Location(267.697, -118.983, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh9_plan_to_start(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the route start of  vehicle 9
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(193.253, -169.608, 0), RoadOption.VOID)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh9_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the remaining route of  vehicle 9
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(193.253, -169.608, 0), RoadOption.STRAIGHT),
                   (carla.Location(209.868, -169.416, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(248.12, -169.032, 0), RoadOption.STRAIGHT),
                   (carla.Location(268.406, -169.224, 0), RoadOption.LANEFOLLOW)]
        # (carla.Location(305.306, -168.84, 0), RoadOption.STRAIGHT),
        # (carla.Location(318.636, -168.845, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh10_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the behavior of  vehicle 10
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(208.612, -172.889, 0), RoadOption.STRAIGHT),
                   (carla.Location(193.748, -173.049, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(137.227, -173.35, 0), RoadOption.STRAIGHT),
                   (carla.Location(120.033, -173.35, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(69.6086, -174.04, 0), RoadOption.STRAIGHT),
                   (carla.Location(51.5313, -173.75, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(18, -174.344, 0), RoadOption.RIGHT),
                   (carla.Location(14.7793, -180.428, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(14.0323, -220.496, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh11_plan_to_start(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the route start of  vehicle 1
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(212.612, -172.889, 0), RoadOption.VOID)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh11_plan_(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the remaining route  of  vehicle 11
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        # follow route of vehicle 10
        return self.get_waypoint_veh10_plan()

    def get_waypoint_veh12_plan_to_start(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the route start of  vehicle 12
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(198.77, -187.4, 0), RoadOption.VOID)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh12_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the remaining route of  vehicle 12
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(198.77, -178.4, 0), RoadOption.STRAIGHT),
                   (carla.Location(198.362, -164.761, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(199.246, -150.7175, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(205.364, -135.187, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(211.346, -127.355, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(218.415, -122.494, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(234.389, -118.848, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(247.916, -118.645, 0), RoadOption.STRAIGHT),
                   (carla.Location(265.697, -118.983, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh13_plan_1(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the route start of  vehicle 13
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(194, -245.204, 0), RoadOption.RIGHT),
                   (carla.Location(198, -239.639, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(198, -181.26, 0), RoadOption.VOID)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh13_plan_2(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the remaining route of  vehicle 13
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(198, -178.61, 0), RoadOption.STRAIGHT),
                   (carla.Location(197.606, -162.249, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(198.38, -156.112, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(200.494, -145.72, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(209.621, -129.69, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(219.304, -121.73, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(232.826, -118.745, 0), RoadOption.LANEFOLLOW),
                   (carla.Location(239.4, -118.954, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def get_waypoint_veh14_plan(self) -> List[Tuple[carla.Waypoint, RoadOption]]:
        """
           The waypoints for the behavior of  vehicle 14
           Returns:
                List[Tuple[carla.Waypoint, RoadOption]]: List of carla.waypoints and RoadOptions
           """
        wp_locs = [(carla.Location(203, -240.179, 0), RoadOption.STRAIGHT),
                   (carla.Location(204, -309.404, 0), RoadOption.LANEFOLLOW)]
        return self.scenario_helper.get_waypoint_plan(wp_locs)

    def _create_test_criteria(self) -> List[Criterion]:
        """
        This method should setup a list with all evaluation criteria for the scenario.
        The criteria should be based on the atomic criteria defined in atomic_scenario_criteria.py.

        Returns:
             List[Criterion]: List of criterions and monitors for this scenario
        """
        loc = carla.Location(203.631, -260.637, 0)
        dist = 4.0
        criteria = [CollisionTest(self.ego_vehicles[0]),
                    ReachedRegionTest(self.ego_vehicles[0], loc.x - dist, loc.x + dist, loc.y - dist, loc.y + dist),
                    OutsideRouteLanesTest(self.ego_vehicles[0], self.get_ego_locations()),
                    # Monitors for each noise level
                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=None, scenario_name=self.scenario_name),
                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 0.1, 2),
                                      scenario_name=self.scenario_name),
                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 0.2, 2),
                                      scenario_name=self.scenario_name),

                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 0.5, 2),
                                      scenario_name=self.scenario_name),
                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 1.0, 2),
                                      scenario_name=self.scenario_name),

                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 2.0, 2),
                                      scenario_name=self.scenario_name),
                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 3.0, 2),
                                      scenario_name=self.scenario_name),
                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 4.0, 2),
                                      scenario_name=self.scenario_name),
                    MonitorEgoVehicle(actor=self.ego_vehicles[0], world=self._world, debug_mode=self._debug_mode,
                                      generate_static_map_info=False, noise=NoiseData([NoiseTypeEnum.POSITION_NOISE],
                                                                                      NoiseTargetEnum.ALL, 0.0, 5.0, 2),
                                      scenario_name=self.scenario_name)
                    ]
        return criteria

    def remove_all_actors(self):
        """
        Removes all actors
        """
        # must delete all ai controller of walkers prior to other objects
        for i, ai in enumerate(self._ai_controllers):
            if CarlaDataProvider.actor_id_exists(ai.id):
                CarlaDataProvider.remove_actor_by_id(ai.id)
            self._ai_controllers[i] = None
        self._ai_controllers = []
        # delelte all other actors
        super().remove_all_actors()

    def get_waypoint_follower_for_actor(self, role_name: str, speed: int,
                                        waypoints: List[Tuple[carla.Waypoint, RoadOption]],
                                        avoid_collision: bool = True) -> WaypointFollower:
        """
            Creates a waypoint follower for an actors using the given List of Waypoints and road option
        Args:
            role_name (): The name of the vehicle
            speed (): the desired veleocity of the vehicle
            waypoints ():  List of Tupels with Waypoint and RoadOptions
            avoid_collision (): activate to avoid coliision with other actors

        Returns:
            WaypointFollwer: The AI to drive the route of the given Waypoints
        """
        return self.scenario_helper.get_waypoint_follower_for_actor(role_name=role_name, actors=self.other_actors,
                                                                    speed=speed, waypoints=waypoints,
                                                                    avoid_collision=avoid_collision)
