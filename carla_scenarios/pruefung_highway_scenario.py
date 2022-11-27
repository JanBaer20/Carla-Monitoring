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


class PruefungHighwayScenario(BasicScenario):
    """
    This class models the scenario "town to highway to town" where the ego vehicle starts on a town road enters the highway and leaves it at the next intersection.

    Starting scenario with:
    $ python3 /opt/scenario_runner/scenario_runner.py --scenario PruefungHighwayScenario --configFile="PruefungHighwayScenario.xml" --additionalScenario="pruefung_highway_scenario.py" --reloadWorld --debug
    """

    def __init__(self, world, ego_vehicles, config, terminate_on_failure=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
           Init function for the class PruefungHighwayScenario

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
        self.scenario_name = "PruefungHighwayScenario"
        self.subtype = config.subtype

        # Call constructor of BasicScenario
        super(PruefungHighwayScenario, self).__init__(
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

        route = [carla.Location(24.84, 109.99, 0),
carla.Location(24.82, 115.53, 0),
carla.Location(24.59, 125.73, 0),
carla.Location(24.52, 135.59, 0),
carla.Location(24.55, 144.62, 0),
carla.Location(24.54, 152.9, 0),
carla.Location(24.61, 163.07, 0),
carla.Location(24.65, 173.39, 0),
carla.Location(21.64, 183.52, 0),
carla.Location(12.57, 188.91, 0),
carla.Location(0.82, 189.94, 0),
carla.Location(-11.82, 190.42, 0),
carla.Location(-25.77, 190.69, 0),
carla.Location(-40.04, 190.83, 0),
carla.Location(-54.58, 190.94, 0),
carla.Location(-68.47, 191.03, 0),
carla.Location(-84.02, 191.18, 0),
carla.Location(-99.06, 191.35, 0),
carla.Location(-114.97, 191.43, 0),
carla.Location(-133.05, 191.34, 0),
carla.Location(-151.14, 190.19, 0),
carla.Location(-169.71, 185.99, 0),
carla.Location(-187.7, 177.88, 0),
carla.Location(-203.74, 165.77, 0),
carla.Location(-216.2, 149.68, 0),
carla.Location(-224.76, 132.08, 0),
carla.Location(-229.42, 112.85, 0),
carla.Location(-229.76, 91.63, 0),
carla.Location(-229.75, 70.64, 0),
carla.Location(-229.24, 49.95, 0),
carla.Location(-228.99, 29.3, 0),
carla.Location(-228.81, 9.69, 0),
carla.Location(-228.62, -9.72, 0),
carla.Location(-228.69, -28.08, 0),
carla.Location(-229.18, -45.45, 0),
carla.Location(-229.66, -62.44, 0),
carla.Location(-230.12, -78.79, 0),
carla.Location(-230.36, -94.63, 0),
carla.Location(-229.33, -111.86, 0),
carla.Location(-225.53, -127.79, 0),
carla.Location(-218.46, -144.05, 0),
carla.Location(-209.74, -156.95, 0),
carla.Location(-198.78, -167.81, 0),
carla.Location(-185.81, -177.11, 0),
carla.Location(-171.7, -184.03, 0),
carla.Location(-158.2, -187.94, 0),
carla.Location(-142.19, -190.1, 0),
carla.Location(-129.02, -190.58, 0),
carla.Location(-114.12, -190.68, 0),
carla.Location(-99.53, -190.71, 0),
carla.Location(-84.7, -190.48, 0),
carla.Location(-70.47, -190.48, 0),
carla.Location(-55.11, -190.53, 0),
carla.Location(-39.13, -190.57, 0),
carla.Location(-23.54, -190.63, 0),
carla.Location(-8.68, -190.62, 0),
carla.Location(4.18, -190.13, 0),
carla.Location(14.77, -187.94, 0),
carla.Location(21.65, -184.9, 0),
carla.Location(26.55, -180.45, 0),
carla.Location(28.95, -173.89, 0),
carla.Location(28.69, -166.26, 0),
carla.Location(28.0, -158.14, 0),
carla.Location(27.68, -149.38, 0),
carla.Location(27.57, -140.01, 0),
carla.Location(27.42, -131.01, 0),
carla.Location(27.27, -121.9, 0),
carla.Location(27.07, -111.36, 0),
carla.Location(26.87, -100.46, 0),
carla.Location(26.6, -89.72, 0),
carla.Location(26.29, -79.15, 0),
carla.Location(25.83, -68.46, 0),
carla.Location(25.37, -57.37, 0),
carla.Location(24.97, -48.19, 0)]
        
        turningPoints = [(carla.Location(24.77, 158.66, 0), "Turn Right"),
(carla.Location(-2.46, -186.8, 0), "Turn Right")]

        # final location for scneario end
        # unfreeze the vehicles to start moving but maintain start position throughout initialization
        loc = carla.Location(25.6, -39.9, 0)
        dist = 4.0
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

        veh3 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh3")

        loc = carla.Location(17.25, 177.4, 0)
        wp = self._world.get_map().get_waypoint(loc)
        tf1 = None
        tf2 = None
        tf3 = None
        for tf in self._world.get_traffic_lights_in_junction(wp.get_junction().id):
            if round(tf.get_transform().location.x, 2) == 17.25:
                tf1 = tf
            elif round(tf.get_transform().location.x, 2) == 15.65:
                tf2 = tf
            elif round(tf.get_transform().location.x, 2) == 40.85:
                tf3 = tf

        loc = carla.Location(38.5, -189.1, 0)
        wp = self._world.get_map().get_waypoint(loc)
        tf4 = None
        tf5 = None
        tf6 = None
        for tf in self._world.get_traffic_lights_in_junction(wp.get_junction().id):
            if round(tf.get_transform().location.x, 2) == 47.95:
                tf4 = tf
            elif round(tf.get_transform().location.x, 2) == 22.75:
                tf5 = tf
            elif round(tf.get_transform().location.x, 2) == 46.35:
                tf6 = tf

        loc = carla.Location(30.5, -88.5, 0)
        wp = self._world.get_map().get_waypoint(loc)
        tf7 = None
        tf8 = None
        tf9 = None
        tf10 = None
        for tf in self._world.get_traffic_lights_in_junction(wp.get_junction().id):
            if round(tf.get_transform().location.x, 2) == 20.25:
                tf7 = tf
            elif round(tf.get_transform().location.x, 2) == 21.25:
                tf8 = tf
            elif round(tf.get_transform().location.x, 2) == 41.4:
                tf9 = tf
            elif round(tf.get_transform().location.x, 2) == 41.85:
                tf10 = tf

        #behavior of traffic_light 1
        tf1_sequence = py_trees.composites.Sequence("Traffic_Light1_Sequence")
        others_node.add_child(tf1_sequence)
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Green))
        tf1_sequence.add_child(Idle(10))
        tf1_sequence.add_child(TrafficLightStateSetter(tf1, carla.TrafficLightState.Red))
        tf1_sequence.add_child(Idle(10))

        #behavior of traffic_light 2
        tf2_sequence = py_trees.composites.Sequence("Traffic_Lighmustang_sequence")
        others_node.add_child(tf2_sequence)
        tf2_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Red))
        tf2_sequence.add_child(Idle(10))
        tf2_sequence.add_child(TrafficLightStateSetter(tf2, carla.TrafficLightState.Green))
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
        tf4_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(-10.0, -189.2, 0), 8.0))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Yellow))
        tf4_sequence.add_child(Idle(5))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Green))
        tf4_sequence.add_child(Idle(5))
        tf4_sequence.add_child(TrafficLightStateSetter(tf4, carla.TrafficLightState.Red))
        tf4_sequence.add_child(Idle(5))

        #behavior of traffic_light 5
        tf5_sequence = py_trees.composites.Sequence("Traffic_Light5_Sequence")
        others_node.add_child(tf5_sequence)
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Green))
        tf5_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(-10.0, -189.2, 0), 8.0))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Yellow))
        tf5_sequence.add_child(Idle(5))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Red))
        tf5_sequence.add_child(Idle(5))
        tf5_sequence.add_child(TrafficLightStateSetter(tf5, carla.TrafficLightState.Green))
        tf5_sequence.add_child(Idle(5))

        #behavior of traffic_light 6
        tf6_sequence = py_trees.composites.Sequence("Traffic_Light6_Sequence")
        others_node.add_child(tf6_sequence)
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Green))
        tf6_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(-10.0, -189.2, 0), 8.0))
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Yellow))
        tf6_sequence.add_child(Idle(5))
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Red))
        tf6_sequence.add_child(Idle(5))
        tf6_sequence.add_child(TrafficLightStateSetter(tf6, carla.TrafficLightState.Green))
        tf6_sequence.add_child(Idle(5))
        
        #behavior of traffic_light 7
        tf7_sequence = py_trees.composites.Sequence("Traffic_Light7_Sequence")
        others_node.add_child(tf7_sequence)
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Green))
        tf7_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(28.0, -160.0, 0), 4.0))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(Idle(2))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Red))
        tf7_sequence.add_child(Idle(8))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Yellow))
        tf7_sequence.add_child(Idle(2))
        tf7_sequence.add_child(TrafficLightStateSetter(tf7, carla.TrafficLightState.Green))
        
        #behavior of traffic_light 8
        tf8_sequence = py_trees.composites.Sequence("Traffic_Light8_Sequence")
        others_node.add_child(tf8_sequence)
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Red))
        tf8_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(28.0, -160.0, 0), 4.0))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Yellow))
        tf8_sequence.add_child(Idle(2))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Green))
        tf8_sequence.add_child(Idle(8))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Yellow))
        tf8_sequence.add_child(Idle(2))
        tf8_sequence.add_child(TrafficLightStateSetter(tf8, carla.TrafficLightState.Red))
        
        #behavior of traffic_light 9
        tf9_sequence = py_trees.composites.Sequence("Traffic_Light9_Sequence")
        others_node.add_child(tf9_sequence)
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Green))
        tf9_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(28.0, -160.0, 0), 4.0))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Yellow))
        tf9_sequence.add_child(Idle(2))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Red))
        tf9_sequence.add_child(Idle(8))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Yellow))
        tf9_sequence.add_child(Idle(2))
        tf9_sequence.add_child(TrafficLightStateSetter(tf9, carla.TrafficLightState.Green))

        #behavior of traffic_light 10
        tf10_sequence = py_trees.composites.Sequence("Traffic_Light10_Sequence")
        others_node.add_child(tf10_sequence)
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Red))
        tf10_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(28.0, -160.0, 0), 4.0))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Yellow))
        tf10_sequence.add_child(Idle(2))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Green))
        tf10_sequence.add_child(Idle(8))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Yellow))
        tf10_sequence.add_child(Idle(2))
        tf10_sequence.add_child(TrafficLightStateSetter(tf10, carla.TrafficLightState.Red))

        # behavior of vehicle 1
        mustang_sequence = py_trees.composites.Sequence("Ford_Mustang_Sequence")
        others_node.add_child(mustang_sequence)
        veh1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh1")
        mustang_sequence.add_child(Idle(5))
        mustang_sequence.add_child(WaypointFollower(veh1, 16,
                                            self.scenario_helper.get_waypoint_plan(locations=[
                                                (carla.Location(1.04, 187.5, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-8.95, 187.62, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-27.27, 187.67, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-48.38, 187.38, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-75.76, 187.64, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-113.3, 187.4, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-137.97, 187.51, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-153.39, 186.46, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-164.91, 183.84, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-175.74, 179.75, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-182.66, 176.25, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-197.34, 166.19, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-207.54, 156.32, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-215.62, 144.91, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-220.85, 133.08, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-224.54, 119.3, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-226.31, 103.34, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-226.15, 88.25, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.95, 76.1, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-226.0, 61.95, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.71, 46.32, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.19, 32.48, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.06, 16.38, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.1, -1.04, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.32, -14.27, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.66, -28.44, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.92, -45.88, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-226.07, -59.62, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-226.4, -76.61, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-226.42, -93.98, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-225.88, -110.02, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-223.45, -124.45, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-218.0, -138.41, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-209.47, -151.68, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-198.89, -162.95, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-185.9, -172.67, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-172.34, -179.57, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-157.97, -183.85, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-140.45, -186.03, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-118.83, -186.08, 0), RoadOption.LANEFOLLOW)]),
                                            avoid_collision=True))
        mustang_sequence.add_child(WaypointFollower(veh1, 8,
                                            self.scenario_helper.get_waypoint_plan(locations=[
                                                (carla.Location(-103.19, -186.01, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-88.78, -186.14, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-73.97, -186.23, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-58.35, -186.31, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-40.67, -186.41, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-27.11, -186.49, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-14.61, -186.56, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-1.15, -186.45, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(6.03, -186.43, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(12.6, -186.48, 0), RoadOption.LANEFOLLOW)]),
                                            avoid_collision=True))
        mustang_sequence.add_child(WaypointFollower(veh1, 10,
                                            self.scenario_helper.get_waypoint_plan(locations=[
                                                (carla.Location(17.36, -186.51, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(19.99, -186.39, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(23.89, -185.64, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(25.65, -184.75, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(27.05, -183.55, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(28.38, -181.65, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(29.06, -179.26, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(29.16, -176.64, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(28.84, -173.92, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(28.6, -171.47, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(28.29, -166.97, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(27.97, -157.58, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(27.58, -145.97, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(27.25, -135.15, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(27.01, -124.67, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(26.84, -115.19, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(26.58, -106.87, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(26.25, -102.84, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(25.73, -100.49, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(24.54, -98.03, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(22.32, -95.8, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(19.84, -94.66, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(16.76, -94.38, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(14.21, -94.55, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(11.07, -94.85, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(6.81, -95.1, 0), RoadOption.LANEFOLLOW),
                                                (carla.Location(-2.73, -95.18, 0), RoadOption.LANEFOLLOW)]),
                                            avoid_collision=True))
        mustang_sequence.add_child(ActorDestroy(veh1))

        # behavior of vehicle 2
        citreon_sequence = py_trees.composites.Sequence("Citreon_Sequence")
        others_node.add_child(citreon_sequence)
        veh2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh2")
        citreon_sequence.add_child(Idle(5))
        citreon_sequence.add_child(WaypointFollower(veh2, 22,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(3.57, 190.74, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-21.06, 191.01, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-47.46, 191.02, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-73.08, 191.47, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-98.53, 191.23, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-119.38, 191.36, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-138.87, 191.22, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-151.83, 190.6, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-164.43, 188.26, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-176.32, 183.87, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-187.85, 177.83, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-198.94, 170.0, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-208.8, 160.4, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-217.25, 148.8, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-223.28, 137.35, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-227.97, 123.51, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-230.03, 110.49, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-230.35, 97.87, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-229.65, 81.78, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-229.17, 64.31, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-229.01, 49.5, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-226.94, 28.39, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-225.49, 13.31, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-225.42, -1.6, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-225.63, -16.75, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-225.79, -31.2, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-226.03, -45.38, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-226.26, -59.65, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-226.3, -72.26, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-226.49, -86.9, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-226.6, -99.71, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-225.96, -112.9, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-223.74, -124.59, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-218.67, -137.07, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-211.46, -148.6, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-203.0, -158.7, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-194.08, -166.85, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-183.03, -174.42, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-171.59, -180.03, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-158.83, -184.06, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-146.49, -186.03, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-133.68, -186.43, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-121.96, -186.52, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-108.91, -186.54, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-96.05, -186.48, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-80.53, -186.34, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-68.1, -186.34, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-54.38, -186.56, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-39.9, -186.63, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-18.06, -186.37, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-4.06, -186.29, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(4.01, -186.27, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(9.11, -186.26, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(17.82, -186.24, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(35.35, -186.28, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(56.56, -186.68, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(74.09, -186.85, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(86.33, -186.75, 0), RoadOption.LANEFOLLOW)]),
                                                    avoid_collision=True, ))
        citreon_sequence.add_child(ActorDestroy(veh2))

        # behavior of vehicle 3
        police_sequence = py_trees.composites.Sequence("Police_Sequence")
        others_node.add_child(police_sequence)
        police_sequence.add_child(WaypointFollower(veh3, 30,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(54.35, 191.5, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(38.79, 192.47, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(19.22, 193.23, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(3.63, 194.16, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-19.48, 194.42, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-39.61, 194.37, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-55.85, 194.88, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-73.24, 195.45, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-97.45, 194.3, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-116.79, 194.44, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-142.69, 194.85, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-164.83, 192.39, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-182.93, 185.8, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-200.73, 173.15, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-215.67, 157.6, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-226.17, 139.41, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-232.2, 118.91, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-233.53, 94.85, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-233.17, 74.08, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-233.1, 53.31, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-232.68, 28.86, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-232.4, 5.41, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-232.75, -15.62, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-232.86, -39.84, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-233.0, -61.89, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-233.56, -83.33, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-232.9, -106.05, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-227.06, -129.64, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-217.72, -146.14, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-207.15, -159.98, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-191.46, -174.87, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-171.94, -184.21, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-150.79, -188.66, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-129.24, -189.71, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-109.75, -189.3, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-84.57, -189.57, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-61.52, -189.82, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-38.14, -189.5, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(-12.27, -189.57, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(16.3, -189.88, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(40.06, -190.11, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(62.75, -190.2, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(84.29, -190.23, 0), RoadOption.LANEFOLLOW),
                                                        (carla.Location(101.93, -190.03, 0), RoadOption.LANEFOLLOW)]),
                                                avoid_collision=True))
        police_sequence.add_child(ActorDestroy(veh3))

        # behavior of vehicle 4
        nissan_sequence = py_trees.composites.Sequence("Nissan_Sequence")
        others_node.add_child(nissan_sequence)
        veh4 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh4")
        nissan_sequence.add_child(WaypointFollower(veh4, 5,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(57.19, 142.07, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(51.83, 141.82, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(45.68, 141.57, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(43.38, 141.51, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        nissan_sequence.add_child(StopVehicle(veh4, 1))
        nissan_sequence.add_child(Idle(2))
        nissan_sequence.add_child(WaypointFollower(veh4, 10,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(42.73, 141.49, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(40.85, 141.19, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(38.35, 139.93, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(36.31, 137.44, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.55, 134.45, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.53, 130.29, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.4, 125.32, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.23, 117.83, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.12, 110.91, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.06, 105.74, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.98, 96.97, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.93, 85.71, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.78, 73.7, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        nissan_sequence.add_child(StopVehicle(veh4, 1))
        # nissan_sequence.add_child(Idle(2))
        # nissan_sequence.add_child(ActorDestroy(veh4))

        # behavior of vehicle 5
        dodge_sequence = py_trees.composites.Sequence("Dodge_Sequence")
        others_node.add_child(dodge_sequence)
        veh5 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh5")
        dodge_sequence.add_child(Idle(2))
        dodge_sequence.add_child(WaypointFollower(veh5, 5,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(15.5, 201.36, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(24.83, 199.1, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(32.54, 191.5, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.48, 180.45, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.98, 170.98, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.98, 162.14, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.94, 158.18, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.01, 148.74, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.92, 136.26, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.92, 123.17, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(34.97, 116.7, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(35.07, 109.13, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        dodge_sequence.add_child(ActorDestroy(veh5))

        # # behavior of vehicle 6
        mercedes_sequence = py_trees.composites.Sequence("Mercedes_Sequence")
        others_node.add_child(mercedes_sequence)
        veh6 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh6")
        mercedes_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(-10.0, -189.2, 0), 8.0))
        mercedes_sequence.add_child(WaypointFollower(veh6, 8,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(33.9, -117.23, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(34.03, -120.12, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(34.5, -134.19, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(35.31, -151.9, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(35.84, -167.32, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(35.95, -178.15, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(35.21, -187.03, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(31.66, -195.22, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(22.61, -202.15, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(12.46, -203.72, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-0.41, -203.61, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-12.46, -203.74, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-25.12, -203.95, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-42.4, -204.0, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-59.91, -203.61, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-78.8, -204.15, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-101.42, -204.11, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(-122.7, -204.28, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True))
        mercedes_sequence.add_child(ActorDestroy(veh6))

        # behavior of vehicle 7
        toyota_sequence = py_trees.composites.Sequence("Toyota_Sequence")
        others_node.add_child(toyota_sequence)
        veh7 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh7")
        toyota_sequence.add_child(Idle(8))
        toyota_sequence.add_child(WaypointFollower(veh7, 18,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(-124.94, 187.8, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-145.97, 187.14, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-169.51, 182.52, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-191.38, 170.65, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-208.41, 154.57, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-222.71, 130.1, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-226.73, 101.46, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-226.46, 75.71, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-226.49, 45.65, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-226.13, 17.88, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-225.64, -10.75, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-225.75, -38.61, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-226.31, -69.35, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-226.37, -114.69, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-222.38, -127.0, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-215.34, -143.57, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-203.8, -159.37, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-185.88, -173.54, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-165.56, -182.59, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-141.71, -186.55, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-117.97, -186.65, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-94.07, -186.99, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-71.08, -187.05, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-51.28, -186.87, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-32.98, -186.75, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-18.66, -186.62, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-7.64, -186.49, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(-0.42, -186.42, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(10.27, -186.61, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(17.84, -186.78, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(32.26, -186.91, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(48.51, -186.9, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(67.94, -186.75, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(83.35, -186.83, 0), RoadOption.STRAIGHT),
                                                    (carla.Location(95.44, -186.98, 0), RoadOption.STRAIGHT)]),
                                                avoid_collision=True))
        toyota_sequence.add_child(ActorDestroy(veh7))

        # # behavior of vehicle 8
        seat_sequence = py_trees.composites.Sequence("Seat_Sequence")
        others_node.add_child(seat_sequence)
        veh8 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "veh8")
        seat_sequence.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0],
                                                               carla.Location(28.0, -160.0, 0), 4.0))
        seat_sequence.add_child(Idle(1))
        seat_sequence.add_child(WaypointFollower(veh8, 12,
                                                    self.scenario_helper.get_waypoint_plan(locations=[
                                                        (carla.Location(30.41, -102.36, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(30.41, -98.7, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(31.01, -94.75, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(33.01, -90.25, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(37.13, -86.34, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(42.29, -84.57, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(48.0, -84.4, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(54.52, -84.83, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(60.3, -84.67, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(66.2, -83.62, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(72.91, -81.3, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(80.0, -77.25, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(86.57, -71.44, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(91.47, -64.65, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(94.47, -57.29, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(95.86, -50.36, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(96.4, -43.72, 0), RoadOption.STRAIGHT),
                                                        (carla.Location(96.65, -37.37, 0), RoadOption.STRAIGHT)]),
                                                    avoid_collision=True))
        seat_sequence.add_child(ActorDestroy(veh8))

        # behavior of pedestrians
        # behavior of pedestrian 1
        ped1_sequence = py_trees.composites.Sequence("Pedestrian1_Sequence")
        others_node.add_child(ped1_sequence)
        ped1 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped1")
        ped1_sequence.add_child(WaypointFollower(ped1, 3,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(38.5059, 131.652, 0), RoadOption.LANEFOLLOW),
                                                    (carla.Location(38.6206, 103.154, 0), RoadOption.LANEFOLLOW)],
                                                    lane_type=carla.LaneType.Sidewalk),
                                                avoid_collision=True))
        ped1_sequence.add_child(ActorDestroy(ped1))

        # behavior of pedestrian 2
        ped2_sequence = py_trees.composites.Sequence("Pedestrian2_Sequence")
        others_node.add_child(ped2_sequence)
        ped2 = self.scenario_helper.get_actor_by_role_name(self.other_actors, "ped2")
        ped2_sequence.add_child(WaypointFollower(ped2, 2,
                                                self.scenario_helper.get_waypoint_plan(locations=[
                                                    (carla.Location(20.4881, 59.9498, 0), RoadOption.LANEFOLLOW)],
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
