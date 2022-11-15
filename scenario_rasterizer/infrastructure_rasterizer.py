#! /usr/bin/env python

from __future__ import annotations

import logging
import math
import sys
from pathlib import Path
from typing import List, Tuple, Set

import ad_map_access as ad
import carla
import numpy as np

from scenario_rasterizer import ScenarioSegment

# logger creation
logger = logging.getLogger(__name__)

# fixed lane types
CONTINUATION_TYPES = {ad.map.lane.LaneType.NORMAL, ad.map.lane.LaneType.EMERGENCY, ad.map.lane.LaneType.MULTI,
                      ad.map.lane.LaneType.OVERTAKING}  # Consider only main lanes for vehicles
DRIVEABLE_LANE_TYPES = {ad.map.lane.LaneType.NORMAL, ad.map.lane.LaneType.EMERGENCY, ad.map.lane.LaneType.MULTI,
                        ad.map.lane.LaneType.OVERTAKING, ad.map.lane.LaneType.TURN, ad.map.lane.LaneType.BIKE}


class InfrastructureRasterizer:
    """
        This class determines the different segments within a long-lasting recording or simulation based on the
        infrastructure - mainly junctions.
        Each junction and normal road (between junctions) results in a single segment.

        It is important to run one of the "analyze_map()" method each time a new map is loaded. Preferally once in the setup.
    """

    # TODO how to DEAL WITH ÜBerführungen und Unterführungen

    def __init__(self):
        """
            Initialization of Rasterizer: Initializes the List of Segments
        """
        self.__scenario_blocks: List[ScenarioSegment] = None

    @property
    def blocks(self) -> List[ScenarioSegment]:
        """
            Gives the segments gathered by the rasterizer
        Returns:
            List[ScenarioSegment]: List of saved segments
        """
        if self.__scenario_blocks is None:
            ## Warning that there are no block.
            logger.warning(
                f'No scenario Blocks for map "{map}"! Have you run "analyze_map()" prior?')

        return self.__scenario_blocks

    def analyze_map_from_xodr_content(self, xodr_content: str, junction_ext: float = 10.0, overlap_margin: float = 0.5,
                                      intersection_type: ad.map.intersection.IntersectionType = ad.map.intersection.IntersectionType.Yield,
                                      light_type: ad.map.landmark.TrafficLightType = ad.map.landmark.TrafficLightType.SOLID_RED_YELLOW_GREEN) \
            -> List[ScenarioSegment]:
        """
            Analyzes the given content of xodr-map for its segments
        Args:
            xodr_content (): the xordr-map
            junction_ext (): extensions distance for each junction
            overlap_margin (): overlaps for the map
            intersection_type (): default type for intersections [optional]
            light_type (): default type for lights at intersections [optional]

        Returns:
            List[ScenarioSegment]: List of analyzed segements for the map
        """
        # init store of the carla map libary
        result = ad.map.access.initFromOpenDriveContent(
            xodr_content, overlap_margin, intersection_type, light_type)

        if not result:
            raise ValueError("Failure while parsing the xodr map.")

        return self.__analyze_map(junction_ext)

    def analyze_map_from_xodr_file(self, xodr_file: str, junction_ext=10.0, overlap_margin=0.5,
                                   intersection_type=ad.map.intersection.IntersectionType.Yield,
                                   light_type=ad.map.landmark.TrafficLightType.SOLID_RED_YELLOW_GREEN) \
            -> List[ScenarioSegment]:
        """
        Args:
            xodr_file (): the file of the xodr-map
            junction_ext (): extensions distance for each junction
            overlap_margin (): overlaps for the map
            intersection_type (): default type for intersections [optional]
            light_type (): default type for lights at intersections [optional]

        Returns:
             List[ScenarioSegment]: List of analyzed segements for the map
        """
        # check if file exists
        if not xodr_file:
            raise ValueError("No xodr file given!")
        # check file type
        xodr_file_path = Path(xodr_file)
        if not xodr_file_path.suffix == '.xodr':
            logger.error(
                f'Give map file {xodr_file} does not have the OpenDrive File Extension ".xodr". Import likely to fail!')

        with open(xodr_file_path, 'r') as file:
            # init store
            result = ad.map.access.initFromOpenDriveContent(
                file.read(), overlap_margin, intersection_type, light_type)

            if not result:
                raise ValueError("Failure while parsing the xodr map.")

            return self.__analyze_map(junction_ext)

    def __check_existing_lane_id(self, lane_id: ad.map.lane.LaneId, analyzed_lane_ids: Set[ad.map.lane.LaneId]) -> bool:
        """
        Checks if the given id of a lane is withn the given set of lane ids analyzed by the rasterizer
        Args:
            lane_id (): lane id to check for
            analyzed_lane_ids (): list of lane ids

        Returns:
            bool: if the lane is within the lane set
        """
        if analyzed_lane_ids is None:
            return False
        if lane_id is None:
            return False

        return str(lane_id) in [str(lane) for lane in analyzed_lane_ids]

    def __add_lane_id(self, lane_id: ad.map.lane.LaneId, id_set: Set[ad.map.lane.LaneId]):
        """
        Only adds to set if the laneID is not available.
        Problem is that laneID is object and not a number or string and therefore not properly compared

        Args:
            lane_id (ad.map.lane.LaneId): the lane to be added
            id_set (Set[ad.map.lane.LaneId]): the set of lane IDs
        """
        if id_set is not None and not str(lane_id) in [str(lane) for lane in id_set]:
            id_set.add(lane_id)

    def __add_lane_ids(self, lane_ids: Set[ad.map.lane.LaneId], target_set: Set[ad.map.lane.LaneId]):
        """
        Only adds to set if the laneID is not available.
        Problem is that laneID is object and not a number or string and therefore not properly compared

        Args:
            laneId (ad.map.lane.LaneId): the lane to be added
            id_set (Set[ad.map.lane.LaneId]): the set of lane IDs
        """
        if target_set is None:
            return
        casted_target_set = [str(l) for l in target_set]
        for lane in lane_ids:
            if not str(lane) in casted_target_set:
                target_set.add(lane)

    def __reduce_LaneID_Set(self, lane_set: Set[ad.map.lane.LaneId], lane_items: Set[ad.map.lane.LaneId]) -> Set[
        ad.map.lane.LaneId]:
        """
        Reduce the lane-set to items not contained in lane items
        Args:
            lane_set (): The set of lane ids to be reduced
            lane_items (): The lane ids to check aausnt

        Returns:
             Set[ad.map.lane.LaneId]: the remaining lane ids in the set
        """
        return {laneID for laneID in lane_set if not str(laneID) in [str(l) for l in lane_items]}

    def __calc_bounding_points(self, laneID: ad.map.lane.LaneId) -> Tuple[ad.map.point.GeoPoint, ad.map.point.GeoPoint]:
        """
            Calculates the bounding box of a given lane
        Args:
            laneID (): the id of the lane to analyze

        Returns:
             Tuple[ad.map.point.GeoPoint, ad.map.point.GeoPoint]: Tuple of min bounding point and max bounding point
        """
        lane = ad.map.lane.getLane(laneID)
        # get border points of lane
        left_edge_geo_points = ad.map.point.toGeo(lane.edgeLeft.ecefEdge)
        right_edge_geo_points = ad.map.point.toGeo(lane.edgeRight.ecefEdge)

        # calculate the min and max of all edge points for the lane
        edge_min = self.__calculate_Edge_Min(
            list(left_edge_geo_points) + list(right_edge_geo_points))
        edge_max = self.__calulcate_edge_max(
            list(left_edge_geo_points) + list(right_edge_geo_points))
        logger.debug(f'{edge_min} - {edge_max}')
        return edge_min, edge_max

    def __out_intersection(self, lane: ad.map.lane.Lane, contactLane: ad.map.lane.Lane) -> bool:
        """
            Determines if the following road is a normal road checking of contactLane is not within the same intersection
            as the given lane
        Args:
            lane ():the given lane within the intersection
            contactLane ():the contact lane to check

        Returns:
            bool: if the contactLane is out of the intersection
        """
        return lane.type is ad.map.lane.LaneType.INTERSECTION and contactLane.type is not ad.map.lane.LaneType.INTERSECTION

    def __into_intersection(self, lane: ad.map.lane.Lane, contactLane: ad.map.lane.Lane) -> bool:
        """
            Determines if the following road is a intersection by checking of contactLane is in the intersection
            based on the given lane

        Args:
            lane (): the given lane within the intersection
            contactLane (): the contact lane to check

        Returns:
             bool: if the contactLane is within an intersection
        """
        return lane.type is not ad.map.lane.LaneType.INTERSECTION and contactLane.type is ad.map.lane.LaneType.INTERSECTION

    def __analyze_lane(self, laneID: ad.map.lane.LaneId, analyzed_lane_ids: Set[ad.map.lane.LaneId] = None,
                       intersection_extension: float = 2.0) -> Tuple[
        ad.map.point.GeoPoint, ad.map.point.GeoPoint, bool, Set[ad.map.lane.LaneId], Set[ad.map.lane.LaneId]]:
        """
            Analyze the given lane by its ID to determine the bounding box of the segment. Recursively analyzes all
            adjacent lanes for the given lane as long as are they in the same road or junction and recursively
            calculates the bounding box with its containing lanes and lanes for further analysis.

        Args:
            laneID (): the lane to be analyzed
            analyzed_lane_ids (): set of already analzed lane
            intersection_extension (): extension factor for an intersection
        Returns:
            Tuple[
        ad.map.point.GeoPoint, ad.map.point.GeoPoint, bool, Set[ad.map.lane.LaneId], Set[ad.map.lane.LaneId]]:
        The current bounding box of the lane's segment with min point and max point, if the segment is a curve,
        the segment's included lanes and the ids of lanes open fir analysis
        """
        if analyzed_lane_ids is None:
            analyzed_lane_ids = set()
        to_analyze_lane_ids = set()
        block_lane_ids = set()
        curvature = False

        logger.debug(f'Analyzing lane {str(laneID)}')

        # get Lane object
        lane = ad.map.lane.getLane(laneID)
        # 2 cases: intersection and normal road

        # calculate bounding box of this lane
        min_pos, max_pos = self.__calc_bounding_points(laneID)

        # dertermine if the lane is curved
        if lane.type in DRIVEABLE_LANE_TYPES:
            curvature = self.__determine_lane_curvature(laneID)

        logger.debug(list(analyzed_lane_ids))
        # add to analyzed lane ids to lane not being analyzed
        self.__add_lane_id(laneID, analyzed_lane_ids)  # this has sideeffeects
        # add analyze lane to the lane within this block
        self.__add_lane_id(laneID, block_lane_ids)

        # start with intersection lanes but bear in mind that there are multiple intersection lanes in one intersection
        contact_lane_list = lane.contactLanes
        logger.debug(
            f'Contact Lanes of {str(laneID)}: {[str(clane) for clane in list(contact_lane_list)]}')
        for clane in contact_lane_list:
            if not self.__check_existing_lane_id(clane.toLane, analyzed_lane_ids):
                logger.debug(
                    f'{clane.location}: {clane.toLane} -  Prohibit: {[str(l) for l in analyzed_lane_ids]}  - To Analyze: {[str(l) for l in to_analyze_lane_ids]}')
                # First look for success and predecessor in order
                if clane.location is ad.map.lane.ContactLocation.SUCCESSOR or \
                        clane.location is ad.map.lane.ContactLocation.PREDECESSOR:
                    logger.debug(
                        f'Found predecessor / successor lane for lane {str(laneID)}')
                    if ad.map.lane.ContactType.LANE_CONTINUATION in clane.types:
                        # only consider continuation and don't work on stop or yield
                        # CHECK IF THE TYPE IS THE SAME -> GET MULTIPLE NORMAL ROADS
                        clane_obj = ad.map.lane.getLane(clane.toLane)
                        if self.__out_intersection(lane, clane_obj) or self.__into_intersection(lane, clane_obj):
                            # these are lanes for new block - add to analyze
                            self.__add_lane_id(clane.toLane, to_analyze_lane_ids)
                        else:
                            # only follow normal roads?
                            if lane.type in CONTINUATION_TYPES:
                                # we remain in the road block
                                _min, _max, _curvature, __block_lanes, _open_lanes = self.__analyze_lane(
                                    clane.toLane, analyzed_lane_ids, intersection_extension)
                                min_pos = self.__calc_min_geo_point(min_pos, _min)
                                max_pos = self.__calc_max_geo_point(max_pos, _max)

                                self.__add_lane_ids(
                                    __block_lanes, block_lane_ids)
                                [to_analyze_lane_ids.add(t) for t in _open_lanes if t not in analyzed_lane_ids]
                # analyze left and right adjacent lanes
                if clane.location is ad.map.lane.ContactLocation.LEFT or clane.location is ad.map.lane.ContactLocation.RIGHT or clane.location is ad.map.lane.ContactLocation.OVERLAP:
                    logger.debug(
                        f'Found adjacenet or overlapping lane for lane {str(laneID)}')
                    _min, _max, _curvature, __block_lanes, _open_lanes = self.__analyze_lane(
                        clane.toLane, analyzed_lane_ids, intersection_extension)
                    min_pos = self.__calc_min_geo_point(min_pos, _min)
                    max_pos = self.__calc_max_geo_point(max_pos, _max)
                    curvature = curvature or _curvature
                    self.__add_lane_ids(__block_lanes, block_lane_ids)
                    [to_analyze_lane_ids.add(t) for t in _open_lanes if t not in analyzed_lane_ids]
                # ALL OTHERS DO NOTHING
        logger.debug(
            f'{min_pos}, {max_pos}, {curvature}, {[str(l) for l in block_lane_ids]}, {[str(l) for l in to_analyze_lane_ids]}')
        return min_pos, max_pos, curvature, block_lane_ids, to_analyze_lane_ids

    def __determine_lane_curvature(self, lane_id: ad.map.lane.LaneId) -> bool:
        """
            This function determines for a given lane id, if the lane is curved based on gradients
            along the edges of the lane.
        Args:
            lane_id (): the id of the lane to investigate

        Returns:
            bool: returns true if the lane is curved, else false
        """

        def get_xytyuple_from_enu_point(point: ad.map.point.ENUPoint) -> (float, float):
            """
            Generates an x,y-Tuple from a given ENUPoint
            Args:
                point ():  the ENUpoint

            Returns:
                (float, float): the tuple of x-coordinate and y-coordinate
            """
            return point.x, point.y

        lane = ad.map.lane.getLane(lane_id)
        # Intersections -> turning lanes are not considered
        # just make a line between the start and end of the edge and see if any point is on this lane
        left_enu_edge = list(map(get_xytyuple_from_enu_point, list(ad.map.point.toENU(lane.edgeLeft.ecefEdge))))
        logger.debug(f'{left_enu_edge}')
        right_enu_edge = list(map(get_xytyuple_from_enu_point, list(ad.map.point.toENU(lane.edgeRight.ecefEdge))))
        logger.debug(f'{right_enu_edge}')
        # usen numpy here for calculations by calulcating the edges gradients
        left_points_nparray = np.array(left_enu_edge, dtype=np.dtype('float'))
        logger.debug(f'{left_points_nparray}')
        right_points_nparray = np.array(right_enu_edge, dtype=np.dtype('float'))
        logger.debug(f'{right_points_nparray}')
        left_points_diff = np.diff(left_points_nparray, axis=0)
        logger.debug(f'{left_points_diff}')
        right_points_diff = np.diff(right_points_nparray, axis=0)
        logger.debug(f'{right_points_diff}')
        # calculates gradients alongside the edges
        left_gradients = (left_points_diff[:, 1] / left_points_diff[:, 0])
        logger.debug(f'{left_gradients}')
        right_gradients = (right_points_diff[:, 1] / right_points_diff[:, 0])
        logger.debug(f'{right_gradients}')
        logger.debug(f'left len = {np.ma.size(left_gradients, axis=0)}')
        logger.debug(f'{np.amin(left_gradients, axis=0)}')
        logger.debug(f'{np.amax(left_gradients, axis=0)}')
        # check if the gradients throughout the lane are identical
        # left edge
        if np.ma.size(left_gradients, axis=0) > 1 and \
                not np.allclose(np.amin(left_gradients, axis=0), np.amax(left_gradients, axis=0)):
            logger.debug(f'Left edge of lane {lane_id} is not a straight line!')
            return True
        logger.debug(f'right len = {np.ma.size(right_gradients, axis=0)}')
        # right edge
        if np.ma.size(right_gradients, axis=0) > 1 and \
                not np.allclose(np.amin(right_gradients, axis=0), np.amax(right_gradients, axis=0)):
            logger.debug(f'Right edge of lane {lane_id} is not a straight line!')
            return True
        logger.debug(f'Lane {lane_id} is straight!')
        return False

    def __analyze_map(self, junction_ext=10.0) -> List[ScenarioSegment]:
        """
            Determines for the imported OpenDrive File all scenario segments using Carla's ad map access library.

            This function should not be called directly. Instead, call [analyze_map_from_xodr_content] or
             analyze_map_from_xodr_file] to first import the OpenDrive File and then call this function.

        Returns:
            [List(ScenarioSegment)]: List of all Scenario Segments for the imported map
        """
        # The list of segments
        segments = []

        # get normal lanes and intersection lane - also includes crosswalks
        lanes_to_analyze = set()
        analyzed_lanes = set()

        # get all lanes in the xodr file
        lane_ids = ad.map.lane.getLanes()
        start_id = lane_ids[0]
        # initate with start laneID
        lanes_to_analyze.add(start_id)

        # iterate over lane to be analyzed
        while lanes_to_analyze:
            logger.debug(
                f'Lanes to analyze: {len(lanes_to_analyze)}')
            lane_id = lanes_to_analyze.pop()
            logger.debug(
                f'Building block with lane {lane_id}: {ad.map.lane.getLane(lane_id)}')
            # recursively analyze lanes in the segment
            _min, _max, _curvature, _block_lane_ids, _open_lanes = self.__analyze_lane(
                lane_id, analyzed_lanes, junction_ext)
            # analyzed_lanes ist updated automatically
            logger.debug(
                f'Block Data from lanes:  {_min}, {_max}, {_curvature}, {_block_lane_ids}, {_open_lanes}')
            self.__add_lane_ids(_open_lanes, lanes_to_analyze)  # add new todos
            # reduce the todos by the one that have been done
            lanes_to_analyze = self.__reduce_LaneID_Set(lanes_to_analyze, analyzed_lanes)
            # create new Scenario Segment
            block = ScenarioSegment(
                carla.GeoLocation(float(_min.latitude), float(_min.longitude), float(_min.altitude)), carla.GeoLocation(
                    float(_max.latitude), float(_max.longitude), float(_max.altitude)), _curvature,
                [str(l) for l in _block_lane_ids])
            # add segments
            segments.append(block)
            logger.debug(f'Block for lane {lane_id}: ({block})')
            logger.debug(
                f'Analzyed Lane IDs: {[str(l) for l in analyzed_lanes]} - Open Lane IDs {[str(l) for l in lanes_to_analyze]}')
            logger.info(
                f'Analzyed Lane IDs: {len(analyzed_lanes)}  // Total Lane IDs: {len(lane_ids)} // Open Lane IDs: {len(lanes_to_analyze)}// Blocks: {len(segments)}')

        logger.debug(f"All identified Blocks: {segments}")
        logger.debug(f'Number of Blocks: {len(segments)}')
        # set that last state if there is a failure don't become a stale state
        self.__scenario_blocks = segments
        return self.__scenario_blocks

    def calculate_scenario_blocks(self, position: carla.GeoLocation, curve_sub_calculation=True) \
            -> List[ScenarioSegment]:
        """
            Calculates all possible scenario blocks in which the given position resides in

        Args:
            position (carla.GeoLocation): The position of an object in the OpenDrive world coordinate system
            curve_sub_calculation (bool): perform sub matching in blocks for curved lanes
        """
        if not self.__scenario_blocks:
            raise ValueError(f'No blocks available! Have you execute methode "analyze_map" to generate the blocks?')

        containing_blocks = list(filter(lambda b: b.is_in(position), self.__scenario_blocks))
        result = list(filter(lambda b: not b.is_curve, containing_blocks))
        if curve_sub_calculation:
            # sub-filtering by distance to lanes
            [result.append(curved_block) for curved_block in filter(lambda b: b.is_curve, containing_blocks)
             if math.isclose(self.__calulcate_road_distance(position, curved_block.lanes), 0.0)]
        if not containing_blocks:
            logger.warning(f' Given Object poistion {position} is not contained in any block.')
        return containing_blocks

    def get_block_for_lane(self, lane_id: int) -> ScenarioSegment | None:
        """
        Returns the block which contains the given lane_id

        Args:
            lane_id (): The lane_id that should be inside the returned block

        Returns:
            The containing Segment or None
        """
        all_blocks = self.blocks
        for block in all_blocks:
            if block.lanes.__contains__(str(lane_id)):
                return block
        return None

    def get_most_probable_block(self, position: carla.GeoLocation, rotation: carla.Rotation,
                                acceptable_delta: float = 0.001, perfect_lane_match: bool = False) \
            -> ScenarioSegment | None:
        """

        Args:
            position (): The GeoLocation of the actor
            rotation (): The rotation of the actor
            acceptable_delta (): The acceptable delta for determining the segment
            perfect_lane_match (): Allow only perfect lane matches

        Returns:
            A Scenario Segment or None
        """
        # first get possible candidates
        _candidates = self.calculate_scenario_blocks(position)
        # if there is not one candidate there will be no match
        if not _candidates:
            return None
        # if there is only one candidate return this candidate
        if len(_candidates) == 1:
            return _candidates[0]
        # with multiple candidates start more detailed matching using AdMapMatching
        map_matcher = ad.map.match.AdMapMatching()
        # use rotation to increase precision of matching by adding mapping hint
        if rotation is not None:
            enu_heading = ad.map.point.createENUHeading(ad.map.point.degree2radians(rotation.yaw))
            map_matcher.addHeadingHint(enu_heading, ad.map.access.getENUReferencePoint())
        # use AdMapMatching to match position to lane positions
        matched_position_confidence_list = map_matcher.getMapMatchedPositions(
            ad.map.point.createGeoPoint(position.longitude, position.latitude, position.altitude), 10.0, 0.0)
        # check heading of object and lane
        logger.debug(f'{list(matched_position_confidence_list)}')
        candidates_min_distances = []
        # find for each block the min distance of its lanes to the candidates
        for candidate in _candidates:
            min_distance = sys.float_info.max
            for lane in candidate.lanes:
                distance = ad.map.match.signedDistanceToLane(int(lane), matched_position_confidence_list)
                distance = abs(float(distance))
                # if we get a direct match with 0 or the acceptably distance is ok
                if math.isclose(distance, 0.0) or math.isclose(distance, acceptable_delta):
                    # if we are on its lane is the most probable due to the previous sorting
                    return candidate
                # go on and look other lane
                min_distance = min(min_distance, distance)
            # look on other candidate
            candidates_min_distances.append(min_distance)
        # result
        if perfect_lane_match:
            # no item found with perfect match
            return None
        else:
            # take the one with min match
            logger.info(f'{candidates_min_distances}')
            block_min_distance = _candidates[candidates_min_distances.index(min(candidates_min_distances))]
            map_matcher.clearHeadingHints()
            return block_min_distance

    def __get_min_geo_point(self) -> ad.map.point.GeoPoint:
        """
            Creates a GeoPOint with Min-Values for altitude, longitude, and latitude to init variables
        Returns:
            ad.map.point.GeoPoint: The GeoPoint with min-values
        """
        geo_point = ad.map.point.GeoPoint()
        geo_point.altitude = ad.map.point.Altitude.getMin()
        geo_point.longitude = ad.map.point.Longitude.getMin()
        geo_point.latitude = ad.map.point.Latitude.getMin()
        return geo_point

    def __get_max_geo_point(self):
        """
           Creates a GeoPOint with Max-Values for altitude, longitude, and latitude to init variables
           Returns:
               ad.map.point.GeoPoint: The GeoPoint with max-values
           """
        geo_point = ad.map.point.GeoPoint()
        geo_point.altitude = ad.map.point.Altitude.getMax()
        geo_point.longitude = ad.map.point.Longitude.getMax()
        geo_point.latitude = ad.map.point.Latitude.getMax()
        return geo_point

    def __calc_min_geo_point(self, that: ad.map.point.GeoPoint, other: ad.map.point.GeoPoint) -> ad.map.point.GeoPoint:
        """
                Determines the overall min GeoPoint for two given GeoPoints.

                Individually calculates the min for latitude, altitude and longitude based on the given GeoPoints
        Args:
            that (): The first GeoPoints
            other (): The second GeoPoint

        Returns:
            ad.map.point.GeoPoint: A GeoPoint with the min for attitude, altitude and longitude
        """
        # check data
        if not that:
            if not other:
                return None
            return other
        if not other:
            return that
        # create empty GeoPoint
        min_point = ad.map.point.GeoPoint()
        min_point.latitude = that.latitude if that.latitude < other.latitude else other.latitude
        min_point.altitude = that.altitude if that.altitude < other.altitude else other.altitude
        min_point.longitude = that.longitude if that.longitude < other.longitude else other.longitude
        return min_point

    def __calc_max_geo_point(self, that: ad.map.point.GeoPoint, other: ad.map.point.GeoPoint) -> ad.map.point.GeoPoint:
        """
                Determines the overall max GeoPoint for two given GeoPoints.

                Individually calculates the max for latitude, altitude and longitude based on the given GeoPoints
        Args:
            that (): The first GeoPoints
            other (): The second GeoPoint

        Returns:
            ad.map.point.GeoPoint: A GeoPoint with the max for attitude, altitude and longitude
        """
        # check data
        if not that:
            if not other:
                return None
            return other
        if not other:
            return that
        # create empty GeoPoint
        max_point = ad.map.point.GeoPoint()
        max_point.latitude = that.latitude if that.latitude > other.latitude else other.latitude
        max_point.altitude = that.altitude if that.altitude > other.altitude else other.altitude
        max_point.longitude = that.longitude if that.longitude > other.longitude else other.longitude
        return max_point

    def __calculate_Edge_Min(self, edge_geo_points: List[ad.map.point.GeoPoint]) -> ad.map.point.GeoPoint:
        """
            Calulcates for a given edge the min-bounding GeoPoint
        Args:
            edge_geo_points (): the edge of a lane given by its GeoPoints

        Returns:
            ad.map.point.GeoPoint: the minimal bounding GeoPint
        """
        # check data
        if not edge_geo_points:
            return None
        min_geo_point = self.__get_max_geo_point()
        for point in edge_geo_points:
            # latitude
            if min_geo_point.latitude > point.latitude:
                min_geo_point.latitude = point.latitude
            # altitude
            if min_geo_point.altitude > point.altitude:
                min_geo_point.altitude = point.altitude
            # longitude
            if min_geo_point.longitude > point.longitude:
                min_geo_point.longitude = point.longitude
        return min_geo_point

    def __calulcate_edge_max(self, edge_geo_points: List[ad.map.point.GeoPoint]) -> ad.map.point.GeoPoint:
        """
             Calulcates for a given edge the max-bounding GeoPoint
         Args:
             edge_geo_points (): the edge of a lane given by its GeoPoints

         Returns:
             ad.map.point.GeoPoint: the maximal bounding GeoPoint
         """
        # check data
        if not edge_geo_points:
            return None
        max_geo_point = self.__get_min_geo_point()
        for point in edge_geo_points:
            # latitude
            if max_geo_point.latitude < point.latitude:
                max_geo_point.latitude = point.latitude
            # altitude
            if max_geo_point.altitude < point.altitude:
                max_geo_point.altitude = point.altitude
            # longitude
            if max_geo_point.longitude < point.longitude:
                max_geo_point.longitude = point.longitude
        return max_geo_point

    def __calulcate_road_distance(self, position: carla.GeoLocation, lane_ids: List[str]) -> float:
        """
            Calculates the minimal distance of the given position to all of the given lane
        Args:
            position (): The GeoLocation as position of an actor
            lane_ids (): The list of lanes by their IDs as strings

        Returns:
            float: The overall minimal distance to the lanes for the given position
        """
        min_distance = None
        # create GeoPosition
        geo_point_position = ad.map.point.createGeoPoint(position.longitude, position.latitude, position.altitude)
        # iterate iver the lanes
        for lane_id in lane_ids:
            lane_distances = []
            # get lane object
            lane = ad.map.lane.getLane(int(lane_id))
            # get botch edges of the lane
            edge_left_geo = ad.map.point.toGeo(lane.edgeLeft.ecefEdge)
            edge_right_geo = ad.map.point.toGeo(lane.edgeRight.ecefEdge)
            # determine the distance from the given position to each point of both edges
            lane_distances = map(lambda p: abs(float(ad.map.point.flatDistance(p, geo_point_position))),
                                 list(edge_left_geo) + list(edge_right_geo))
            # get the overall minimal distance
            lane_distances_min = min(lane_distances)
            # save minimal distance for lane if it is the overall minimal distance for all lanes
            if min_distance is not None:
                min_distance = min(min_distance, lane_distances_min)
            else:
                min_distance = lane_distances_min
        # Check if we have any minimal distance
        if min_distance == sys.float_info.max:
            logger.error(f'Did not have any lanes with reasonable distance to given position {position}!')
        return min_distance


