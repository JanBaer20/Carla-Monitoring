#! /usr/bin/env python
import json
import logging
import os
import string
from enum import Enum
from typing import List, Tuple, Dict

import numpy as np
from numpy.typing import ArrayLike
from srunner.scenariomanager.timer import GameTime

from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataLocation import DataLocation
from carla_data_classes.EnhancedJSONEncoder import EnhancedJSONEncoder

logger = logging.getLogger(__name__)


class NoiseTypeEnum(Enum):
    NO_NOISE = 0,
    ALL_NOISE = 1,
    POSITION_NOISE = 2,
    ROTATION_NOISE = 3,
    VELOCITY_NOISE = 4,
    ACCELERATION_NOISE = 5,
    ANGULAR_VELOCITY_NOISE = 6,


def get_noise(shape: Tuple[int, int], mean: float = 0.0, deviation: float = 0.1):
    return np.random.normal(mean, deviation, shape)


def add_position_noise(actors: List[DataActor], noise: ArrayLike) -> Dict:
    """
    Adds noise to the position of given Carla Actors
    """
    logger.debug(f'adding noise to location of actors')
    noise_actors = align_noise_3d_shape(actors, noise)
    data_dict = {'type': 'position', 'values': []}
    for actor, pos_noise in zip(actors, noise_actors):
        actor_dict = {'actor': actor.id, 'init_value': actor.location}
        logger.debug(f'Initial position of actor {actor.id} is {actor.location}.')
        location_noise = DataLocation(None, pos_noise[0], pos_noise[1], pos_noise[2])
        logger.debug(f'Noise for actors {actor.id} is {location_noise}.')
        actor.location = DataLocation(None,
                                      actor.location.x + location_noise.x,
                                      actor.location.y + location_noise.y,
                                      actor.location.z + location_noise.z)
        logger.debug(f'New location of actor {actor.id} is {actor.location}')
        actor_dict['noise_value'] = actor.location
        data_dict['values'].append(actor_dict)
    return data_dict


def add_rotation_noise(actors: List[DataActor], noise: ArrayLike) -> Dict:
    """
    Adds noise to the rotation of given carla actors.
    """
    logger.debug(f'adding noise to rotation of actors')
    noise_actors = align_noise_3d_shape(actors, noise)
    data_dict = {'type': 'rotation', 'values': []}
    for actor, pos_noise in zip(actors, noise_actors):
        logger.debug(f'Initial rotation of actor {actor.id} is {actor.rotation}.')
        actor_dict = {'actor': actor.id, 'init_value': actor.rotation}
        logger.debug(f'Noise for actors {actor.id} is {pos_noise}.')
        actor.rotation.pitch = (actor.rotation.pitch + pos_noise[0]) % 360
        actor.rotation.yaw = (actor.rotation.yaw + pos_noise[1]) % 360
        actor.rotation.roll = (actor.rotation.roll + pos_noise[2]) % 360
        logger.debug(f'New rotation of actor {actor.id} is {actor.rotation}')
        actor_dict['noise_value'] = actor.rotation
        data_dict['values'].append(actor_dict)
    return data_dict


def add_velocity_noise(actors: List[DataActor], noise: ArrayLike) -> Dict:
    """
    Add noise to the velocitiy of actors
    """
    logger.debug(f'adding noise to velocity of actors')
    noise_actors = align_noise_3d_shape(actors, noise)
    data_dict = {'type': 'velocity', 'values': []}
    for actor, pos_noise in zip(actors, noise_actors):
        logger.debug(f'Initial velocity of actor {actor.id} is {actor.velocity}.')
        actor_dict = {'actor': actor.id, 'init_value': actor.velocity}
        logger.debug(f'Noise for actors {actor.id} is {pos_noise}.')
        actor.velocity.x = actor.velocity.x + pos_noise[0]
        actor.velocity.y = actor.velocity.y + pos_noise[1]
        actor.velocity.z = actor.velocity.z + pos_noise[2]
        logger.debug(f'New velocity of actor {actor.id} is {actor.velocity}')
        actor_dict['noise_value'] = actor.velocity
        data_dict['values'].append(actor_dict)
    return data_dict


def add_acceleration_noise(actors: List[DataActor], noise: ArrayLike) -> Dict:
    logger.debug(f'adding noise to acceleration of actors')
    noise_actors = align_noise_3d_shape(actors, noise)
    data_dict = {'type': 'acceleration', 'values': []}
    for actor, pos_noise in zip(actors, noise_actors):
        logger.debug(f'Initial acceleration of actor {actor.id} is {actor.acceleration}.')
        actor_dict = {'actor': actor.id, 'init_value': actor.acceleration}
        logger.debug(f'Noise for actors {actor.id} is {pos_noise}.')
        actor.acceleration.x = actor.acceleration.x + pos_noise[0]
        actor.acceleration.y = actor.acceleration.y + pos_noise[1]
        actor.acceleration.z = actor.acceleration.z + pos_noise[2]
        logger.debug(f'New acceleration of actor {actor.id} is {actor.acceleration}')
        actor_dict['noise_value'] = actor.acceleration
        data_dict['values'].append(actor_dict)
    return data_dict


def add_angular_velocity_noise(actors: List[DataActor], noise: ArrayLike) -> Dict:
    logger.debug(f'adding noise to angular velocity of actors')
    noise_actors = align_noise_3d_shape(actors, noise)
    data_dict = {'type': 'angular_velocity', 'values': []}
    for actor, pos_noise in zip(actors, noise_actors):
        logger.debug(f'Initial angular velocity of actor {actor.id} is {actor.angular_velocity}.')
        actor_dict = {'actor': actor.id, 'init_value': actor.angular_velocity}
        logger.debug(f'Noise for actors {actor.id} is {pos_noise}.')
        actor.angular_velocity.x = actor.angular_velocity.x + pos_noise[0]
        actor.angular_velocity.y = actor.angular_velocity.y + pos_noise[1]
        actor.angular_velocity.z = actor.angular_velocity.z + pos_noise[2]
        logger.debug(f'New  angular velocity of actor {actor.id} is {actor.angular_velocity}')
        actor_dict['noise_value'] = actor.angular_velocity
        data_dict['values'].append(actor_dict)
    return data_dict


def add_forward_vector_noise(actors: List[DataActor], noise: ArrayLike) -> Dict:
    logger.debug(f'adding noise to forward vector of actors')
    noise_actors = align_noise_3d_shape(actors, noise)
    data_dict = {'type': 'forward_vector', 'values': []}
    for actor, pos_noise in zip(actors, noise_actors):
        logger.debug(f'Initial angular velocity of actor {actor.id} is {actor.forward_vector}.')
        actor_dict = {'actor': actor.id, 'init_value': actor.forward_vector}
        logger.debug(f'Noise for actors {actor.id} is {pos_noise}.')
        actor.forward_vector.x = actor.forward_vector.x + pos_noise[0]
        actor.forward_vector.y = actor.forward_vector.y + pos_noise[1]
        actor.forward_vector.z = actor.forward_vector.z + pos_noise[2]
        logger.debug(f'New forward vector of actor {actor.id} is {actor.forward_vector}')
        actor_dict['noise_value'] = actor.forward_vector
        data_dict['values'].append(actor_dict)
    return data_dict


def align_noise_3d_shape(actors: List[DataActor], noise: ArrayLike) -> ArrayLike:
    if len(actors) == 0:
        return []
    if noise is None or len(noise) == 0 or np.shape(noise)[1] < 0 or np.shape(noise)[1] > 3:
        raise ValueError(f'Dimension for noise not suitable for actor position')
    if np.shape(noise)[0] is not len(actors):
        raise ValueError(f'Noise Shape {np.shape(noise.shape)} does not match actors size ({len(actors)})')
    final_noise = np.zeros((len(actors), 3))
    final_noise[:noise.shape[0], :noise.shape[1]] = noise
    return final_noise


def apply_normal_noise(actors: List[DataActor], noise_type=None, mean: float = 0.0, deviation: float = 0.1,
                       shape: int = 2, log_data: bool = False, log_folder: string = None, file_pattern: string = None,
                       reorder_data: bool = True) -> None:
    data = []
    if noise_type is None:
        noise_type = [NoiseTypeEnum.NO_NOISE]
    if NoiseTypeEnum.ALL_NOISE in noise_type or NoiseTypeEnum.POSITION_NOISE in noise_type:
        changes = add_position_noise(actors, get_noise((len(actors), shape), mean, deviation))
        data.append(changes)
    if NoiseTypeEnum.ALL_NOISE in noise_type or NoiseTypeEnum.ROTATION_NOISE in noise_type:
        changes = add_rotation_noise(actors, get_noise((len(actors), shape), mean, deviation))
        data.append(changes)
    if NoiseTypeEnum.ALL_NOISE in noise_type or NoiseTypeEnum.VELOCITY_NOISE in noise_type:
        changes = add_velocity_noise(actors, get_noise((len(actors), shape), mean, deviation))
        data.append(changes)
    if NoiseTypeEnum.ALL_NOISE in noise_type or NoiseTypeEnum.ACCELERATION_NOISE in noise_type:
        changes = add_acceleration_noise(actors, get_noise((len(actors), 2), mean, deviation))
        data.append(changes)
    if NoiseTypeEnum.ALL_NOISE in noise_type or NoiseTypeEnum.ANGULAR_VELOCITY_NOISE in noise_type:
        changes = add_angular_velocity_noise(actors, get_noise((len(actors), 2), mean, deviation))
        data.append(changes)
    if log_data:
        log_position_change(data, log_folder, file_pattern, reorder_data)


def log_position_change(changes, log_folder: string, file_name: string, reorder_data: bool = True) -> None:
    current_tick = round(GameTime.get_time(), 2)
    logger.debug(changes)

    def change_data_order(data):
        ordered_changes = {}
        for block in data:
            for values in block['values']:
                # actor_block = ordered_changes.get(values['actor'])
                change = {'init_value': values['init_value'], 'noise_value': values['noise_value']}
                # if actor_block is None:# CREATE NEW
                actor_block = ordered_changes.get(values['actor'])
                if actor_block is None:
                    ordered_changes[values['actor']] = {block['type']: change}
                else:
                    actor_block[block['type']] = change
        return ordered_changes

    if reorder_data:
        changes = change_data_order(changes)

    if not os.path.exists(log_folder):
        os.makedirs(log_folder)
    # Create path for log file
    file = f"{file_name}".replace(":", "-").replace(" ", "_").replace(".", "-")
    file = os.path.join(log_folder, f"{file}.json")
    with open(file, "a") as logfile:
        json.dump({current_tick: changes}, logfile, cls=EnhancedJSONEncoder)
