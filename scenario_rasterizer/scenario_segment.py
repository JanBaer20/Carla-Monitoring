#! /usr/bin/env python

from dataclasses import dataclass, field
from typing import List

from carla import GeoLocation


@dataclass(frozen=True, unsafe_hash=True)
class ScenarioSegment(object):
    """
        The 'ScenarioSegment' data class encapsulates a block of the world from the given by the Open Drive file
        representing a possible part of the single scenario
    """
    lower_bound: GeoLocation  # lower corner
    higher_bound: GeoLocation  # higher corner
    is_curve: bool = field(compare=False, default=False)  # does segment contain a curved road
    lanes: List[str] = field(default_factory=list)  # list of lanes in the segment
    id: str = field(init=False)  # the id

    def __post_init__(self):
        """
            Called after initialization to calculate the ID automatically
        """
        if self.lower_bound == GeoLocation(-1, -1, -1) and \
                self.higher_bound == GeoLocation(-1, -1, -1) and \
                self.is_curve is False and self.lanes == [-1]:
            object.__setattr__(self, 'id', -1)
        else:
            # Use string of lanes instead of hash because hash has not been unique!
            id_str = "-".join(sorted(self.lanes))
            object.__setattr__(self, 'id', id_str)

    def is_in(self, location: GeoLocation, check_altitude: bool = False) -> bool:
        """
            Evaluates if an object - given by its location - is within the segments bounding box - ignoring its
             altitude [optional]
        Args:
            location ():  location of the object
            check_altitude (): active the consideration of the altiude for mathicn

        Returns:
            bool:
        """
        # check lower bound
        if location.latitude < self.lower_bound.latitude or \
                location.longitude < self.lower_bound.longitude:
            return False
        # check high bound
        if location.latitude > self.higher_bound.latitude or \
                location.longitude > self.higher_bound.longitude:
            return False
        # the case if we explicitly need to check the altitude
        if check_altitude and (location.altitude < self.lower_bound.altitude or
                               location.altitude > self.higher_bound.altitude):
            return False
        return True

    def __repr__(self):
        """
            Overwrite of string represenation
        Returns:
            str: String representation of an Scenario Segment
        """
        return f'ScenarioSegment(Id: {self.id} Min:{self.lower_bound}, Max:{self.higher_bound}, Curve:{self.is_curve}, Lanes: {[str(l) for l in self.lanes]})'

    @staticmethod
    def get_null_block():
        """
            Creates an empty Scenario Segment
        Returns:
            ScenarioSegment
        """
        return ScenarioSegment(GeoLocation(-1, -1, -1), GeoLocation(-1, -1, -1), False, ['-1'])
