from time import sleep
import py_trees
from carla import TrafficLightState
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion
from srunner.scenariomanager.timer import GameTime

import carla
from carla_helpers.world_helper import WorldHelper

import argparse
import socket

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65000  # The port used by the server


class VehiclePassedRedTrafficLightTest(Criterion):
    """
    This class contains an atomic test for vehicles that pass red traffic lights.
    """

    MAX_IDLE_TIME = 1  # Amount of time in seconds until a vehicle is flagged as "stopped"
    MIN_VELOCITY = 0.5  # Amount of speed necessary to start the test

    def __init__(self, actor, debug_mode, name="VehiclePassedRedTrafficLightTest", terminate_on_failure=False):
        super(VehiclePassedRedTrafficLightTest, self).__init__(name, actor, 0,
                                                               terminate_on_failure=terminate_on_failure)

        self.logger.debug("%s.__init__()" % self.__class__.__name__)

        self._actor = actor
        self._debug_mode = debug_mode
        self._world_helper = WorldHelper(actor.get_world())
        # Save starting point as "stopped"
        self._last_stopped_time = GameTime.get_time()
        # Save if the vehicle has moved once
        self._has_initially_moved = False
        # Initialize list to store already tracked stop times
        self._stopped_times = []
        self._already_passed_traffic_light = None

    # Gets called each tick and checks the implemented test
    def update(self):
        """
        Check for each traffic light if the vehicle passed it while it was in the red state
        :return: Current status of the test
        """
        if self._debug_mode:
            print(f"Current time: {GameTime.get_time()}")

        # Saves the status of the current update cycle of the VehicleStoppedTest
        new_test_status = py_trees.common.Status.RUNNING

        actor_speed = self._actor.get_velocity()
        # Get the current speed of the ego vehicle
        actor_speed = CarlaDataProvider.get_velocity(self._actor)

        if self._debug_mode:
            print(f"Current velocity: {actor_speed}")

        # Get all traffic lights for the lane the vehicle is currently on
        traffic_lights = self._world_helper.get_traffic_lights_for_vehicle(self._actor)
        if len(traffic_lights) == 0:
            self._already_passed_traffic_light = None
            if self._debug_mode:
                print("There is currently no traffic light affecting the vehicle")
            return py_trees.common.Status.RUNNING

        if self._debug_mode:
            print(f"Traffic lights affecting the vehicle: {traffic_lights}")

        # Get nearest waypoint of the actor
        nearest_waypoint = self._world_helper.get_nearest_waypoint_for_actor(self._actor)

        if self._debug_mode:
            print(f"Current waypoint: {nearest_waypoint}")

        for traffic_light in traffic_lights:
            if traffic_light.state == TrafficLightState.Red and self._already_passed_traffic_light != traffic_light:
                self._already_passed_traffic_light = traffic_light
                # The ego vehicle stopped long enough. Set test status
                self.send_violation(b"Vehicle drove over red traffic light")
                self.test_status = py_trees.common.Status.FAILURE

                # Record event by increasing actual result by 1
                self.actual_value += 1

        # Terminate test if the "terminate on failure" flag is set and the current status if "FAILURE"
        if self._terminate_on_failure and (self.test_status == py_trees.common.Status.FAILURE):
            new_test_status = py_trees.common.Status.FAILURE

        # Log update cycle information
        self.logger.debug("%s.update()[%s->%s]" % (self.__class__.__name__, self.status, new_test_status))

        return new_test_status


    def send_violation(self, msg):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            self._socket = s
            s.sendall(msg)
            s.close()

def monitor(args):
    world = None
    print(args.actor)

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    world = client.get_world()

    GameTime.restart()
    possible_vehicles = world.get_actors().filter(args.filter)
    for vehicle in possible_vehicles:
        if vehicle.attributes['role_name'] == 'hero' and args.actor == 'ego':
            actor = vehicle
        elif vehicle.attributes['role_name'] == args.actor:
            actor = vehicle
    
    CarlaDataProvider.set_client(client)
    CarlaDataProvider.register_actor(actor)

    test = VehiclePassedRedTrafficLightTest(actor, debug_mode=True,
                                                terminate_on_failure=args.sof)

    while True:
        snapshot = world.get_snapshot()
        if snapshot:
            timestamp = snapshot.timestamp
            GameTime.on_carla_tick(timestamp)
        
        CarlaDataProvider.on_carla_tick()
        test.update()
        sleep(1/int(args.Hz))

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-a', '--actor',
        action='store',
        default='ego',
        dest='actor',
        help='actor name (default ego vehicle)')
    argparser.add_argument(
        '--sof',
        action='store_true',
        dest='sof',
        help='Stop on failure')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '-Hz',
        action='store',
        default=1,
        dest='Hz',
        help='Tickrate of the execution')
    args = argparser.parse_args()

    print("Test")
    try:
        print("Test")
        monitor(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':

    main()