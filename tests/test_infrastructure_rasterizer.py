from __future__ import annotations

import math
import unittest
from subprocess import Popen
from typing import List, Tuple

import carla

from scenario_rasterizer import ScenarioSegment, InfrastructureRasterizer

MAP_FILE_PATH = '../resources/tests/Town04.xodr'
_carla_process: Popen


class InfrastructureRasterizerTest(unittest.TestCase):
    _rasterizer: InfrastructureRasterizer = None


    def setUp(self) -> None:
        self._rasterizer = InfrastructureRasterizer()

    def test_load_town04(self):
        analyze_result = self._rasterizer.analyze_map_from_xodr_file(MAP_FILE_PATH, junction_ext=2.0)
        self.assertEqual(len(analyze_result), 82, 'The number of blocks for Town04 is not correct')
        # question the rasterizer the feedback
        result_blocks = self._rasterizer.blocks
        self.assertEqual(len(result_blocks), 82, 'The number of blocks for Town04 is not correct')
        self.assertTrue(self._compare_block_lists(analyze_result, result_blocks))

    def test_calculate_scenario_blocks(self):
        analyze_result = self._rasterizer.analyze_map_from_xodr_file(MAP_FILE_PATH, junction_ext=2.0)
        self.assertTrue(len(analyze_result) > 0)
        position_results = self._get_object_positions()
        for position, blocks in position_results:
            all_blocks = self._rasterizer.calculate_scenario_blocks(position, curve_sub_calculation=False)
            self.assertEqual(len(all_blocks), len(blocks)),
            self._compare_block_lists(all_blocks, blocks, delta=0.1)

            no_curve_blocks = self._rasterizer.calculate_scenario_blocks(position, curve_sub_calculation=True)
            self.assertLessEqual(len(no_curve_blocks), len(all_blocks))

    def test_calculate_most_probable_block(self):
        analyze_result = self._rasterizer.analyze_map_from_xodr_file(MAP_FILE_PATH, junction_ext=2.0)
        self.assertTrue(len(analyze_result) > 0)
        position_results = self._get_object_positions()
        for position, blocks in position_results:
            block = self._rasterizer.get_most_probable_block(position, carla.Rotation(0.0, 0.0, 0.0))
            self.assertIsNotNone(block)
            self._compare_blocks(block, analyze_result[0])

    def _compare_blocks(self, first: ScenarioSegment, second: ScenarioSegment,
                        delta: float = 0.002) -> bool:
        if first.is_curve != second.is_curve or first.lanes.sort() != second.lanes.sort() or \
                not self._compare_geolocation(first.lower_bound, second.lower_bound, delta) or \
                not self._compare_geolocation(first.higher_bound, second.higher_bound, delta):
            return False
        else:
            return True

    def _compare_block_lists(self, first: List[ScenarioSegment], second: List[ScenarioSegment],
                             delta: float = 0.002) -> bool:
        if len(first) != len(second):
            return False
        for fblock in first:
            found = False
            for sblock in second:
                if self._compare_blocks(fblock, sblock, delta):
                    found = True
                    break
            if not found:
                return False
        return True

    @staticmethod
    def _get_object_positions() -> List[Tuple[carla.GeoLocation, List[ScenarioSegment]]]:
        # We cannot match the Scenarioblock by ID because the number is not deterministic
        positions = []
        geo1 = carla.GeoLocation(latitude=0.001531, longitude=0.000377, altitude=0.197355)
        # block 29
        block29 = ScenarioSegment(carla.GeoLocation(latitude=0.001521, longitude=0.000225, altitude=0.195892),
                                  carla.GeoLocation(latitude=0.001600, longitude=0.000431, altitude=0.195892),
                                  is_curve=False, lanes=['70151', '70152', '70153', '70148', '70149'])
        positions.append((geo1, [block29]))

        geo2 = carla.GeoLocation(latitude=0.001062, longitude=0.002632, altitude=0.021197)
        # block 74
        block74 = ScenarioSegment(carla.GeoLocation(latitude=0.001048, longitude=0.002401, altitude=0.019592),
                                  carla.GeoLocation(latitude=0.001149, longitude=0.002715, altitude=0.019592),
                                  is_curve=False,
                                  lanes=['190149', '190153', '190151', '190154', '190148', '190152'])
        positions.append([geo2, [block74]])

        geo3 = carla.GeoLocation(latitude=0.000586, longitude=0.002942, altitude=0.006155)
        # block 77
        block77 = ScenarioSegment(carla.GeoLocation(latitude=0.000559, longitude=0.002767, altitude=0.004349),
                                  carla.GeoLocation(latitude=0.001001, longitude=0.003329, altitude=0.004349),
                                  is_curve=True, lanes=['50151', '50148', '50152', '50149'])
        positions.append([geo3, [block77]])

        geo4 = carla.GeoLocation(latitude=0.000044, longitude=-0.000903, altitude=9.313180)  #

        # block 7
        block7 = ScenarioSegment(carla.GeoLocation(latitude=-0.000536, longitude=-0.001086, altitude=8.699016),
                                 carla.GeoLocation(latitude=0.000143, longitude=-0.000607, altitude=9.841583),
                                 is_curve=False, lanes=['11910146', '11850153', '11940154', '11840148',
                                                        '11910145', '11910248', '11850154', '11840145',
                                                        '11940151', '11840147', '11940253', '11850155',
                                                        '11850152', '11940152', '11910148', '11940251',
                                                        '11940153', '11850157', '11840149', '11910247',
                                                        '11910147', '11940155', '11910149', '11850151',
                                                        '11940156', '11840146', '11940157', '11940252',
                                                        '11910249', '11850156'])
        positions.append([geo4, [block7]])

        geo5 = carla.GeoLocation(latitude=-0.002878, longitude=-0.000400, altitude=0.001953)
        # block 13, 12
        block13 = ScenarioSegment(carla.GeoLocation(latitude=-0.003212, longitude=-0.000514, altitude=0.000000),
                                  carla.GeoLocation(latitude=-0.002469, longitude=0.000252, altitude=0.000000),
                                  is_curve=False, lanes=['1450148', '1440151', '1440152', '1440156', '1450147',
                                                         '1440153', '1440154', '1350149', '1450146', '1450145',
                                                         '1360149', '1450149', '1440157', '1440155'])
        block12 = ScenarioSegment(carla.GeoLocation(latitude=-0.003991, longitude=-0.004676, altitude=0.000000),
                                  carla.GeoLocation(latitude=-0.000006, longitude=-0.000215, altitude=0.000000),
                                  is_curve=True, lanes=['60156', '410146', '60151', '450147', '410155', '60149',
                                                        '410147', '60148', '450154', '410151', '410145', '60147',
                                                        '450156', '450152', '60146', '450151', '450155', '60145',
                                                        '450153', '410153', '450148', '60154', '410148',
                                                        '450145',
                                                        '60152', '60157', '410152', '450157', '60155', '410154',
                                                        '410156', '450149', '410157', '450146', '60153',
                                                        '410149'])
        positions.append([geo5, [block13, block12]])

        geo6 = carla.GeoLocation(latitude=-0.003565, longitude=-0.003311, altitude=0.001955)
        # block12
        positions.append([geo6, [block12]])

        geo7 = carla.GeoLocation(latitude=-0.000809, longitude=0.001159, altitude=8.095208)
        # block 27,76
        block27 = ScenarioSegment(carla.GeoLocation(latitude=-0.001365, longitude=0.000281, altitude=-0.000000),
                                  carla.GeoLocation(latitude=-0.000509, longitude=0.001209, altitude=9.949390),
                                  is_curve=True, lanes=['310152', '310153', '310151'])
        block76 = ScenarioSegment(carla.GeoLocation(latitude=-0.002713, longitude=0.000191, altitude=0.000000),
                                  carla.GeoLocation(latitude=-0.000364, longitude=0.003650, altitude=0.000000),
                                  is_curve=True, lanes=['220149', '220147', '220146', '220148'])
        positions.append([geo7, [block27, block76]])
        return positions

    @staticmethod
    def _compare_geolocation(first: carla.GeoLocation, second: carla.GeoLocation, delta: float = 0.0):
        if first == second:
            return True
        else:
            if delta == 0.0:
                return False
            if math.isclose(first.longitude, second.longitude, abs_tol=delta) and \
                    math.isclose(first.latitude, second.latitude, abs_tol=delta) and \
                    math.isclose(first.altitude, second.altitude, abs_tol=delta):
                return True
            else:
                return False


# just for later usage
# def tearDownClass(cls) -> None:
#     global _carla_process
#     if _carla_process is None:
#         # kill carla process by name
#         processes = Popen(['ps', '-A'], stdout=PIPE)
#         output, error = processes.communicate()
#
#         target_process = "Carla"
#         [os.kill(int(line.split(None, 1)[0]), 9) for line in output.splitlines() if target_process in str(line)]
#     else:
#         # kill carla by process
#         _carla_process.terminate()


if __name__ == '__main__':
    print('test')
    unittest.main()
