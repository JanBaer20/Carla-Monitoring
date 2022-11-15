import logging
import unittest
from builtins import ValueError
from typing import List
from unittest.mock import Mock

import numpy as np
from carla import GeoLocation, Location, Vector3D, Rotation, Actor

from carla_data_classes.DataActor import DataActor
from carla_data_classes.DataLocation import DataLocation
from carla_data_classes.DataRotation import DataRotation
from carla_data_classes.DataVector3D import DataVector3D
from carla_noise_generator import normal_noise_generator
from carla_noise_generator.normal_noise_generator import NoiseTypeEnum

logger = logging.getLogger(__name__)


class BasicNoiseGenerationTest(unittest.TestCase):

    def generate_carla_actor_mocks(self, num: int) -> List[Actor]:
        if num < 1:
            raise ValueError('Number of actors to generate must at least be 1')
        actors = []
        # Build mocks for actors
        for i in range(1, num + 1):
            actor = Mock()
            actor.id = i
            actor.ad_map_lane_id = i
            actor.road_id = i
            actor.lane_id = i
            actor.geo_location = GeoLocation(i, i, i)
            actor.location = Location(i, i, i)
            actor.location.x = i
            actor.location.y = i
            actor.location.z = i
            actor.get_location.return_value = Location(i, i, i)
            actor.get_location.x = i
            actor.get_location.y = i
            actor.get_location.z = i
            actor.rotation = Rotation(i, i, i)
            actor.get_transform.return_value.rotation = Rotation(i, i, i)
            actor.get_transform.return_value.get_rotation.return_value = Rotation(i, i, i)
            actor.velocity = Vector3D(i, i, i)
            actor.get_velocity.return_value = Vector3D(i, i, i)
            actor.acceleration = Vector3D(i, i, i)
            actor.get_acceleration.return_value = Vector3D(i, i, i)
            actor.angular_velocity = Vector3D(i, i, i)
            actor.get_angular_velocity.return_value = Vector3D(i, i, i)
            actor.forward_vector = Vector3D(i, i, i)
            actor.get_transform.return_value.get_forward_vector.return_value = Vector3D(i, i, i)
            actors.append(actor)
        return actors


class PositionNoiseTest(BasicNoiseGenerationTest):

    def test_add_position_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        self.assertEqual(np.shape(noise), (len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_position_noise(actors, noise)
        self.assertEqual(actors[0].location, add_datalocations(carla_actors[0].get_location(), noise[0]))
        self.assertEqual(actors[1].location, add_datalocations(carla_actors[1].get_location(), noise[1]))

    def test_add_position_small_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 2))
        self.assertEqual(np.shape(noise), (len(actors), 2))
        logger.debug(f'{noise}')
        normal_noise_generator.add_position_noise(actors, noise)
        self.assertEqual(actors[0].location, add_datalocations(carla_actors[0].get_location(), noise[0]))
        self.assertEqual(actors[1].location, add_datalocations(carla_actors[1].get_location(), noise[1]))

    def test_add_position_larger_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 4))
        self.assertEqual(np.shape(noise), (len(actors), 4))
        logger.debug(f'{noise}')
        with self.assertRaises(ValueError):
            normal_noise_generator.add_position_noise(actors, noise)
        # self.assertRaises(Exception, normal_noise_generator.add_position_noise(actors, noise))

    def test_add_position_no_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = []
        with self.assertRaises(ValueError):
            normal_noise_generator.add_position_noise(actors, noise)

    def test_add_position_no_actors(self):
        actors = []
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_position_noise(actors, noise)
        self.assertCountEqual(actors, [])


class RotiationNoiseTest(BasicNoiseGenerationTest):
    def test_add_rotation_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        self.assertEqual(np.shape(noise), (len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_rotation_noise(actors, noise)
        self.assertEqual(actors[0].rotation, add_datarotation(carla_actors[0].get_transform().rotation, noise[0]))
        self.assertEqual(actors[1].rotation, add_datarotation(carla_actors[1].get_transform().rotation, noise[1]))

    def test_add_rotation_small_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 2))
        self.assertEqual(np.shape(noise), (len(actors), 2))
        logger.debug(f'{noise}')
        normal_noise_generator.add_rotation_noise(actors, noise)
        self.assertEqual(actors[0].rotation, add_datarotation(carla_actors[0].get_transform().rotation, noise[0]))
        self.assertEqual(actors[1].rotation, add_datarotation(carla_actors[1].get_transform().rotation, noise[1]))

    def test_add_rotation_larger_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 4))
        self.assertEqual(np.shape(noise), (len(actors), 4))
        logger.debug(f'{noise}')
        with self.assertRaises(ValueError):
            normal_noise_generator.add_rotation_noise(actors, noise)

    def test_add_rotation_no_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = []
        with self.assertRaises(ValueError):
            normal_noise_generator.add_rotation_noise(actors, noise)

    def test_add_rotation_no_actors(self):
        actors = []
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_rotation_noise(actors, noise)
        self.assertCountEqual(actors, [])


class VelocityNoiseTest(BasicNoiseGenerationTest):
    def test_add_velocity_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        self.assertEqual(np.shape(noise), (len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_velocity_noise(actors, noise)
        self.assertEqual(actors[0].velocity, add_3d_vector(carla_actors[0].get_velocity(), noise[0]))
        self.assertEqual(actors[1].velocity, add_3d_vector(carla_actors[1].get_velocity(), noise[1]))

    def test_add_velocity_small_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 2))
        self.assertEqual(np.shape(noise), (len(actors), 2))
        logger.debug(f'{noise}')
        normal_noise_generator.add_velocity_noise(actors, noise)
        self.assertEqual(actors[0].velocity, add_3d_vector(carla_actors[0].get_velocity(), noise[0]))
        self.assertEqual(actors[1].velocity, add_3d_vector(carla_actors[1].get_velocity(), noise[1]))

    def test_add_velocity_larger_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 4))
        self.assertEqual(np.shape(noise), (len(actors), 4))
        logger.debug(f'{noise}')
        with self.assertRaises(ValueError):
            normal_noise_generator.add_velocity_noise(actors, noise)

    def test_add_velocity_no_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = []
        with self.assertRaises(ValueError):
            normal_noise_generator.add_velocity_noise(actors, noise)

    def test_add_velocity_no_actors(self):
        actors = []
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_velocity_noise(actors, noise)
        self.assertCountEqual(actors, [])


class AccelerationNoiseTest(BasicNoiseGenerationTest):
    def test_add_acceleration_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        self.assertEqual(np.shape(noise), (len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_acceleration_noise(actors, noise)
        self.assertEqual(actors[0].acceleration, add_3d_vector(carla_actors[0].get_acceleration(), noise[0]))
        self.assertEqual(actors[1].acceleration, add_3d_vector(carla_actors[1].get_acceleration(), noise[1]))

    def test_add_acceleration_small_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 2))
        self.assertEqual(np.shape(noise), (len(actors), 2))
        logger.debug(f'{noise}')
        normal_noise_generator.add_acceleration_noise(actors, noise)
        self.assertEqual(actors[0].acceleration, add_3d_vector(carla_actors[0].get_acceleration(), noise[0]))
        self.assertEqual(actors[1].acceleration, add_3d_vector(carla_actors[1].get_acceleration(), noise[1]))

    def test_add_acceleration_larger_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 4))
        self.assertEqual(np.shape(noise), (len(actors), 4))
        logger.debug(f'{noise}')
        with self.assertRaises(ValueError):
            normal_noise_generator.add_acceleration_noise(actors, noise)

    def test_add_acceleration_no_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = []
        with self.assertRaises(ValueError):
            normal_noise_generator.add_acceleration_noise(actors, noise)

    def test_add_acceleration_no_actors(self):
        actors = []
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_acceleration_noise(actors, noise)
        self.assertCountEqual(actors, [])


class AngularVelocityNoiseTest(BasicNoiseGenerationTest):

    def test_add_angular_velocity_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        self.assertEqual(np.shape(noise), (len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_angular_velocity_noise(actors, noise)
        self.assertEqual(actors[0].angular_velocity, add_3d_vector(carla_actors[0].get_angular_velocity(), noise[0]))
        self.assertEqual(actors[1].angular_velocity, add_3d_vector(carla_actors[1].get_angular_velocity(), noise[1]))

    def test_add_angular_velocity_small_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 2))
        self.assertEqual(np.shape(noise), (len(actors), 2))
        logger.debug(f'{noise}')
        normal_noise_generator.add_angular_velocity_noise(actors, noise)
        self.assertEqual(actors[0].angular_velocity, add_3d_vector(carla_actors[0].get_angular_velocity(), noise[0]))
        self.assertEqual(actors[1].angular_velocity, add_3d_vector(carla_actors[1].get_angular_velocity(), noise[1]))

    def test_add_angular_velocity_larger_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 4))
        self.assertEqual(np.shape(noise), (len(actors), 4))
        logger.debug(f'{noise}')
        with self.assertRaises(ValueError):
            normal_noise_generator.add_angular_velocity_noise(actors, noise)

    def test_add_angular_velocity_no_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        noise = []
        with self.assertRaises(ValueError):
            normal_noise_generator.add_angular_velocity_noise(actors, noise)

    def test_add_angular_velocity_no_actors(self):
        actors = []
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.add_angular_velocity_noise(actors, noise)
        self.assertCountEqual(actors, [])


class NormalNoiseGeneratorTest(BasicNoiseGenerationTest):
    def test_apply_all_noise_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.ALL_NOISE], mean, deviation)
        self.assert_noised_attributes(actors[0], carla_actors[0], deviation=.3)
        self.assert_noised_attributes(actors[1], carla_actors[1], deviation=.3)

    def test_apply_all_noise_small_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, mean=mean, deviation=deviation)
        self.assert_noised_attributes(actors[0], carla_actors[0], deviation=.3)
        self.assert_noised_attributes(actors[1], carla_actors[1], deviation=.3)

    def test_apply_all_noise_larger_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        with self.assertRaises(ValueError):
            normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.ALL_NOISE], mean=0.0, deviation=0.1,
                                                      shape=4)

    def test_apply_position_noise_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.POSITION_NOISE], mean, deviation)
        np.testing.assert_allclose(toList(actors[0].location), toList(carla_actors[0].get_location()), atol=.3)
        self.assertListEqual(toList(actors[0].rotation), toList(carla_actors[0].get_transform().rotation))
        self.assertListEqual(toList(actors[0].velocity), toList(carla_actors[0].get_velocity()))
        self.assertListEqual(toList(actors[0].acceleration), toList(carla_actors[0].get_acceleration()))
        self.assertListEqual(toList(actors[0].angular_velocity), toList(carla_actors[0].get_angular_velocity()))

    def test_apply_rotation_noise_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.ROTATION_NOISE], mean, deviation)
        np.testing.assert_allclose(toList(actors[0].rotation), toList(carla_actors[0].get_transform().rotation),
                                   atol=.3)
        self.assertListEqual(toList(actors[0].location), toList(carla_actors[0].get_location()))
        self.assertListEqual(toList(actors[0].velocity), toList(carla_actors[0].get_velocity()))
        self.assertListEqual(toList(actors[0].acceleration), toList(carla_actors[0].get_acceleration()))
        self.assertListEqual(toList(actors[0].angular_velocity), toList(carla_actors[0].get_angular_velocity()))

    def test_apply_velocity_noise_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.VELOCITY_NOISE], mean, deviation)
        np.testing.assert_allclose(toList(actors[0].velocity), toList(carla_actors[0].get_velocity()),
                                   atol=.3)
        self.assertListEqual(toList(actors[0].location), toList(carla_actors[0].get_location()))
        self.assertListEqual(toList(actors[0].rotation), toList(carla_actors[0].get_transform().rotation))
        self.assertListEqual(toList(actors[0].acceleration), toList(carla_actors[0].get_acceleration()))
        self.assertListEqual(toList(actors[0].angular_velocity), toList(carla_actors[0].get_angular_velocity()))

    def test_apply_acceleration_noise_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.ACCELERATION_NOISE], mean, deviation)
        np.testing.assert_allclose(toList(actors[0].acceleration), toList(carla_actors[0].get_acceleration()), atol=.3)
        self.assertListEqual(toList(actors[0].location), toList(carla_actors[0].get_location()))
        self.assertListEqual(toList(actors[0].rotation), toList(carla_actors[0].get_transform().rotation))
        self.assertListEqual(toList(actors[0].velocity), toList(carla_actors[0].get_velocity()))
        self.assertListEqual(toList(actors[0].angular_velocity), toList(carla_actors[0].get_angular_velocity()))

    def test_apply_angelur_velcoity_noise_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.ANGULAR_VELOCITY_NOISE], mean, deviation)
        np.testing.assert_allclose(toList(actors[0].angular_velocity), toList(carla_actors[0].get_angular_velocity()),
                                   atol=.3)
        self.assertListEqual(toList(actors[0].location), toList(carla_actors[0].get_location()))
        self.assertListEqual(toList(actors[0].rotation), toList(carla_actors[0].get_transform().rotation))
        self.assertListEqual(toList(actors[0].velocity), toList(carla_actors[0].get_velocity()))
        self.assertListEqual(toList(actors[0].acceleration), toList(carla_actors[0].get_acceleration()))

    def test_apply_position_velocity_noise_exact_noise(self):
        carla_actors = self.generate_carla_actor_mocks(2)
        actors = [DataActor(actor, ad_map_lane_id=actor.ad_map_lane_id,
                            lane_id=actor.lane_id, road_id=actor.road_id,
                            geo_location=actor.geo_location)
                  for actor in carla_actors]
        logger.debug(f'{actors}')
        mean = 0.0
        deviation = 0.1
        normal_noise_generator.apply_normal_noise(actors, [NoiseTypeEnum.POSITION_NOISE, NoiseTypeEnum.VELOCITY_NOISE],
                                                  mean,
                                                  deviation)
        np.testing.assert_allclose(toList(actors[0].location), toList(carla_actors[0].get_location()),
                                   atol=.3)
        np.testing.assert_allclose(toList(actors[0].velocity), toList(carla_actors[0].get_velocity()),
                                   atol=.3)
        self.assertListEqual(toList(actors[0].rotation), toList(carla_actors[0].get_transform().rotation))
        self.assertListEqual(toList(actors[0].angular_velocity), toList(carla_actors[0].get_angular_velocity()))
        self.assertListEqual(toList(actors[0].acceleration), toList(carla_actors[0].get_acceleration()))

    def test_apply_all_noise_no_actors(self):
        actors = []
        logger.debug(f'{actors}')
        noise = normal_noise_generator.get_noise((len(actors), 3))
        logger.debug(f'{noise}')
        normal_noise_generator.apply_normal_noise(actors)
        self.assertCountEqual(actors, [])

    @staticmethod
    def assert_noised_attributes(actual: DataActor, initial: Actor, deviation: float) -> None:
        np.testing.assert_allclose(toList(actual.location), toList(initial.get_location()), atol=deviation)
        np.testing.assert_allclose(toList(actual.rotation), toList(initial.get_transform().rotation), atol=deviation)
        np.testing.assert_allclose(toList(actual.acceleration), toList(initial.get_acceleration()), atol=deviation)
        np.testing.assert_allclose(toList(actual.angular_velocity), toList(initial.get_angular_velocity()),
                                   atol=deviation)
        np.testing.assert_allclose(toList(actual.velocity), toList(initial.get_velocity()), atol=deviation)


def add_datalocations(location: Location, noise) -> DataLocation:
    new_loc = DataLocation(None, location.x, location.y, location.z)
    if noise is None or len(noise) == 0:
        raise ValueError('Noise has no entries')
    new_loc.x = new_loc.x + noise[0]
    if len(noise) > 1:
        new_loc.y = new_loc.y + noise[1]
    if len(noise) > 2:
        new_loc.z = new_loc.z + noise[2]
    return new_loc


def add_datarotation(rotation: Rotation, noise) -> DataRotation:
    new_rot = DataRotation(rotation)
    if noise is None or len(noise) == 0:
        raise ValueError('Noise has no entries')
    new_rot.pitch = (new_rot.pitch + noise[0]) % 360
    if len(noise) > 1:
        new_rot.yaw = (new_rot.yaw + noise[1]) % 360
    if len(noise) > 2:
        new_rot.roll = (new_rot.roll + noise[2]) % 360
    return new_rot


def add_3d_vector(vector: Vector3D, noise) -> DataVector3D:
    new_vec = DataVector3D(vector)
    if noise is None or len(noise) == 0:
        raise ValueError('Noise has no entries')
    new_vec.x = new_vec.x + noise[0]
    if len(noise) > 1:
        new_vec.y = new_vec.y + noise[1]
    if len(noise) > 2:
        new_vec.z = new_vec.z + noise[2]
    return new_vec


def toList(object) -> []:
    if type(object) is DataRotation or type(object) is Rotation:
        return [object.pitch, object.yaw, object.roll]
    else:
        return [object.x, object.y, object.z]


if __name__ == '__main__':
    unittest.main()
