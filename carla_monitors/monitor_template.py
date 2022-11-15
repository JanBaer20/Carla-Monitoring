from time import sleep
#import py_trees
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


class MonitorName(Criterion):
    """
    This class contains an atomic test for vehicles that pass red traffic lights.
    """

    MAX_IDLE_TIME = 1  # Amount of time in seconds until a vehicle is flagged as "stopped"
    MIN_VELOCITY = 0.5  # Amount of speed necessary to start the test

    def __init__(self, actor, debug_mode, name="MonitorName", terminate_on_failure=False):
        super(MonitorName, self).__init__(name, actor, 0,
                                                               terminate_on_failure=terminate_on_failure)

        self.logger.debug("%s.__init__()" % self.__class__.__name__)

        self._actor = actor
        self._debug_mode = debug_mode
        self._world_helper = WorldHelper(actor.get_world())

        # Any further declarations

    # Gets called each tick and checks the implemented test
    def update(self):
        """
        Description of the Monitor
        """

        if self._debug_mode:
            print(f"Current time: {GameTime.get_time()}")

        # All tests are performed here


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

    test = MonitorName(actor, debug_mode=True, terminate_on_failure=args.sof)

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