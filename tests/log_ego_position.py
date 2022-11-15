#! /usr/bin/env python
import glob
import logging
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

logger = logging.getLogger(__name__)


def print_ego_position(hostname: str = 'localhost', host_port: int = 2000):
    client = carla.Client(hostname, host_port)
    client.set_timeout(10.0)  # seconds

    world = client.get_world()
    map = world.get_map()

    actor_list = world.get_actors()
    logger.debug(f'Size of actors: {len(actor_list)}')

    vehicles = actor_list.filter('vehicle.*')
    logger.debug(f'{len(vehicles)}')
    logger.debug(f'{vehicles}')

    if not vehicles:
        logger.error('no actors available')

    ego_vehicles = list(filter(lambda s: s.attributes['role_name'] == 'hero', vehicles))
    logger.info(ego_vehicles)

    if len(ego_vehicles) == 0:
        logger.error("Ego Vehicle not found")
        return

    if len(ego_vehicles) > 1:
        logger.error("More than one Ego Vehicle found")
        return

    # found exactly one ego vehicle to work with
    logger.debug(f'"Ego is: {ego_vehicles[0]}')

    while True:
        ego_location = ego_vehicles[0].get_location()
        logger.info(f'Ego Location: {ego_location}')
        ego_geo = map.transform_to_geolocation(ego_location)
        logger.info(f'GeoLocaiton of Ego: {ego_geo}')


if __name__ == "__main__":
    logging.basicConfig(format='%(module)s - %(levelname)s: %(message)s', level=logging.INFO)

    print_ego_position()
