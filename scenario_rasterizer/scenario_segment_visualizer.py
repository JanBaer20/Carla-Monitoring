from __future__ import annotations

import logging
import os
import random
from argparse import ArgumentParser
from typing import List, Tuple, Any

import ad_map_access as ad
import carla
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pandas import DataFrame

from carla_data_classes.DataLane import DataLane
from scenario_rasterizer import ScenarioSegment, InfrastructureRasterizer

# logger definition
logger = logging.getLogger(__name__)

# definition of colors
RED = carla.Color(255, 0, 0)
GREEN = carla.Color(0, 255, 0)
BLUE = carla.Color(47, 210, 231)
CYAN = carla.Color(0, 255, 255)
YELLOW = carla.Color(255, 255, 0)
ORANGE = carla.Color(255, 162, 0)
WHITE = carla.Color(255, 255, 255)


class ScenarioSegmentVisualizer:
    """
        This class allows to visualize the scenario segment calculated by the InfrastructureRasterizer to be visualized
        within a Carla simulation and as figure for a given image of the simulation's map.
    """

    # constants for string charset  and debug lifetime
    STRING_CHAR_OFFSET = 0.5
    DEBUG_LIFETIME = 30.0

    def __init__(self, carla_server: str = "localhost", carla_port: int = 2000, client_timeout: float = 10.0):
        """
            Initializes the connection to the carla sever to visualize the blocks in the carla simulation

        Args:
            carla_server (): the address of the Carla server
            carla_port (): the post of the Carla server
            client_timeout ():  The timeout for the connection to the Carla server in seconds
        """
        self._server = carla_server
        self._port = carla_port
        self._time_out = client_timeout
        self._client = None
        self._carla_debugger = None

    def visualize_segments_in_carla(self, segments: List[ScenarioSegment],
                                    debug_lifespan: float = DEBUG_LIFETIME, altitude_offset=7.0,
                                    draw_block_ids: bool = True, line_width: float = 0.1,
                                    color: carla.Color = carla.Color(random.randrange(100, 255), 0, 0, 0)) -> None:
        """
            This methods draws the bounding boxes of all given Scenario Segment within the Carla Simulations.

            Requires a connection to an active Carla simulation!
        Args:
            segments (): The scenario segments to be visualized
            debug_lifespan (): The life span of the visualization objects in the simulation
            altitude_offset (): an offset to be added to the altitude value of all points in the simulations
            draw_block_ids ():  activate the output of the blocks ids
            line_width ():  the width of the line drawn in the simulation
            color ():  the line color for the bounding boxes of each segment in the simulation
        """
        if segments is None:
            logger.error("No blocks for Visualization")
            segments = []

        # Configure the connection to the carla server
        self._client = carla.Client(self._server, self._port)
        self._client.set_timeout(self._time_out)  # seconds
        self._carla_debugger = self._client.get_world().debug

        # draw the given segments
        self.__draw_segments(segments, debug_lifespan, altitude_offset, draw_block_ids=draw_block_ids,
                             line_width=line_width, color=color)

    def draw_lanes(self, lanes: List[DataLane], life_time: float = DEBUG_LIFETIME, z_offset: float = 7.0,
                   color: carla.Color = carla.Color(random.randrange(100, 255), 0, 0, 0)) -> None:
        """
            Draws the set of DataLanes with their IDs in the Carla simulation.

            Requires a connection to an active Carla simulation!
        Args:
            lanes (): The Datalanes to be visualized in the simulation
            life_time (): The life span of the visualization objects in the simulation
            z_offset ():  an offset to be added to the altitude value in the simulations
            color (): the font color
        """
        for lane in lanes:
            self.draw_lane(lane, life_time=life_time, z_offset=z_offset, color=color)

    def draw_lane(self, data_lane: DataLane, life_time=DEBUG_LIFETIME, z_offset: float = 7.0,
                  color: carla.Color = carla.Color(random.randrange(100, 255), 0, 0, 0)) -> None:
        """
            Draws the given DataLane with its ID in the Carla simulation.

            Requires a connection to an active Carla simulation!
        Args:
            data_lane (): The lane to be visualized in the simulation
            life_time (): The life span of the visualization objects in the simulation
            z_offset ():  an offset to be added to the altitude value in the simulations
            color (): the font color
        """
        if self._client is None:
            self._client = carla.Client(self._server, self._port)
            self._client.set_timeout(self._time_out)  # seconds
            self._carla_debugger = self._client.get_world().debug

        for midpoint_index in range(len(data_lane.lane_midpoints) - 1):
            # Get starting midpoint
            from_midpoint = data_lane.lane_midpoints[midpoint_index]
            # Get next midpoint
            to_midpoint = data_lane.lane_midpoints[midpoint_index + 1]
            # Draw line between the two midpoints
            self._carla_debugger.draw_line(from_midpoint.to_location(), to_midpoint.to_location(),
                                           life_time=life_time, color=color)

        # Get median midpoint index
        median_midpoint_index = int(np.median(range(len(data_lane.lane_midpoints))))
        # Get midpoint at median index
        median_midpoint = data_lane.lane_midpoints[median_midpoint_index]

        # Draw debug string at median midpoint of the given lane
        debug_string = f"{data_lane.ad_map_lane_id}, {data_lane.direction}"
        offset_point = median_midpoint
        offset_point.z = offset_point.z + z_offset
        self._carla_debugger.draw_string(offset_point.to_location(),
                                         debug_string, draw_shadow=True, color=color, life_time=life_time)

    def __convert_geolocation_to_enulocation(self, geolocation: carla.GeoLocation) -> carla.Location:
        """
            Converts a given Geolocation into an carla.Location
        Args:
            geolocation (): The given position as GeoLocation

        Returns:
            carla.Location (): The position as carla.Location (ENU)
        """
        # create GeoPoint in ad_map_access
        geo_point = ad.map.point.GeoPoint()
        geo_point.latitude = - geolocation.latitude
        geo_point.altitude = geolocation.altitude
        geo_point.longitude = geolocation.longitude
        # Convert to ENU
        enu_point = ad.map.point.toENU(geo_point)
        logger.debug(f'Create Enu Point {enu_point}')
        # return carla.Location
        return carla.Location(float(enu_point.x), float(enu_point.y), float(enu_point.z))

    def __draw_segments(self, segments: List[ScenarioSegment], lifetime: float = 1000,
                        altitude_offset: float = 7.0, draw_block_ids: bool = True, line_width=0.1,
                        color: carla.Color = carla.Color(random.randrange(100, 255), 0, 0, 0)) -> None:
        """
            The functions draws for each segments its bounding box and it ID (if desired) in the Carla Simulation.
        Args:
            segments (): List of Scenario Segments to be visualized
            lifetime ():  The life span of the visualization objects in the simulation
            altitude_offset (): an offset to be added to the altitude (z) value of all points in the simulations
            draw_block_ids (): activate the output of the blocks ids
            line_width ():  the width of the line drawn in the simulation
            color ():  the line color for the bounding boxes of each segment in the simulation
        """
        logger.info(f'Drawing {len(segments)} blocks in Carla')
        for i, block in enumerate(segments):
            # convert geolocations from segments to enulocation
            p11 = self.__convert_geolocation_to_enulocation(block.lower_bound)
            p24 = self.__convert_geolocation_to_enulocation(block.higher_bound)
            # determine the bounding box points by adding the altitude offset - otherwise we left with quadrants
            p24.z = p24.z + altitude_offset
            p12 = carla.Location(p11.x, p24.y, p11.z)
            p13 = carla.Location(p24.x, p11.y, p11.z)
            p14 = carla.Location(p24.x, p24.y, p11.z)
            p21 = carla.Location(p11.x, p11.y, p24.z)
            p22 = carla.Location(p11.x, p24.y, p24.z)
            p23 = carla.Location(p24.x, p11.y, p24.z)
            # draw the lines between the direct corners of bounding box
            self._carla_debugger.draw_line(p11, p12, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p11, p13, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p12, p14, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p13, p14, color=color, life_time=lifetime, thickness=line_width)

            self._carla_debugger.draw_line(p11, p21, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p12, p22, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p13, p23, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p14, p24, color=color, life_time=lifetime, thickness=line_width)

            self._carla_debugger.draw_line(p21, p22, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p21, p23, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p22, p24, color=color, life_time=lifetime, thickness=line_width)
            self._carla_debugger.draw_line(p23, p24, color=color, life_time=lifetime, thickness=line_width)

            # draw the segment ID in the bounding box center
            if draw_block_ids:
                id_txt = f'{i} ({block.id})'
                self._carla_debugger.draw_string(
                    carla.Location(p11.x + ((p24.x - p11.x) / 2) - (len(id_txt) * self.STRING_CHAR_OFFSET),
                                   p11.y + ((p24.y - p11.y) / 2), p24.z),
                    id_txt, draw_shadow=True, color=color, life_time=1000 if lifetime == 0.0 else lifetime)

    def visualize_segments_as_plot(self, segments: List[ScenarioSegment], path_to_bg_image: str) -> None:
        """
            Visualizes the given segments in the given image of the simulated map
        Args:
            segments (): Scenario segments to be visualized
            path_to_bg_image (): path to the image of the simulated map as background for the figure
        """
        # Using  pandas dataframe for easier visualization in the figure
        df = self.__get_dataframe(segments)
        logger.debug(df.head())

        plt.title('Plotting scenario blocks')

        # initializing the scatter plot
        plt.scatter(df.p1longitude, df.p1latitude, zorder=1, c='b', s=5)
        plt.scatter(df.p2longitude, df.p2latitude, zorder=1, c='b', s=5)
        plt.scatter(df.p3longitude, df.p3latitude, zorder=1, c='b', s=5)
        plt.scatter(df.p4longitude, df.p4latitude, zorder=1, c='b', s=5)

        # determine the max size of all bounding boxes and match that to the background
        bb = self.__calc_segments_bounding_box(df)
        logger.debug(bb)
        plt.xlim(bb[0], bb[2])
        plt.ylim(bb[1], bb[3])

        # draw a rectangle for each segment with a rolling number
        for i, block in enumerate(segments):
            rect = patches.Rectangle((block.lower_bound.longitude, block.lower_bound.latitude),
                                     block.higher_bound.longitude - block.lower_bound.longitude,
                                     block.higher_bound.latitude - block.lower_bound.latitude,
                                     fill=False,
                                     color="purple",
                                     linewidth=5)
            logger.info(
                f'Block {i}:({block.id},{block.lower_bound},{block.higher_bound}, {block.is_curve}) represented by {rect} // Lanes in Block: {[str(l) for l in block.lanes]}')
            # add patch wit rolling number
            plt.gca().add_patch(rect)
            plt.annotate(str(i), self.__calc_label_position(block))

        # show the boxes on the given background image
        if not os.path.exists(path_to_bg_image):
            logger.warning('No background image given for plotting')
        else:
            bg_path = os.path.abspath(path_to_bg_image)
            map_img = plt.imread(bg_path)
            plt.imshow(map_img, zorder=0, extent=(bb[0], bb[2], bb[1], bb[3]), aspect='auto')
        logger.debug('Finished Visualization')
        plt.show()

    def __get_dataframe(self, segments: List[ScenarioSegment]) -> pd.DataFrame:
        """
            Builds a Panda data frame from the given scenario segments. Each point in the dataframe consists
            of the eight vertices of the bounding box.
        Args:
            segments (): The segments to be visualized

        Returns:
            DataFrame: The dataframe of the bounding vertices of all given segments
        """
        # convert segments into tuples of its bounding vertices
        points = [(block.lower_bound.latitude, block.lower_bound.longitude,
                   block.higher_bound.latitude, block.lower_bound.longitude,
                   block.lower_bound.latitude, block.higher_bound.longitude,
                   block.higher_bound.latitude, block.higher_bound.longitude) for block in segments]
        # build data frame
        df = pd.DataFrame(points,
                          columns=['p1latitude', 'p1longitude', 'p2latitude', 'p2longitude', 'p3latitude',
                                   'p3longitude', 'p4latitude', 'p4longitude'])
        # output to csv
        df.to_csv('./block_dataframe_backup.csv', sep=";")
        return df

    def __calc_segments_bounding_box(self, df: DataFrame) -> Tuple[Any, Any, Any, Any]:
        """
            Calculates the overall bounding Box for all points in the given DataFrame
        Args:
            df (): the data frame with the bounding points of all segements

        Returns:
            Tuple[Any, Any, Any, Any]: The min_latitude,min_longitude,max_latitude,max_longitude   of the overall bounding box
        """
        # calculate min latitude
        min_latitude = df.p1latitude.min()
        p4_lat_min = df.p4latitude.min()
        min_latitude = p4_lat_min if min_latitude > p4_lat_min else min_latitude

        # calculate min longitude
        min_longitude = df.p1longitude.min()
        p4_long_min = df.p4longitude.min()
        min_longitude = p4_long_min if min_longitude > p4_long_min else min_longitude

        # calculate max latitude
        max_latitude = df.p1latitude.max()
        p4_lat_max = df.p4latitude.max()
        max_latitude = p4_lat_max if max_latitude > p4_lat_max else max_latitude

        # calculate max longitude
        max_longitude = df.p1longitude.max()
        p4_long_max = df.p4longitude.max()
        max_longitude = p4_long_max if max_longitude > p4_long_max else max_longitude

        # return tuple
        return min_longitude, min_latitude, max_longitude, max_latitude

    def __calc_label_position(self, block: ScenarioSegment) -> Tuple[float, float]:
        """
            Calculates the position of the segements label in its bounding box's center
        Args:
            block (): The block to visualize

        Returns:
            Tuple[float, float]: The x,y- position of the label
        """
        # determine distance between corners of bounding box
        delta_x = block.higher_bound.longitude - block.lower_bound.longitude
        delta_y = block.higher_bound.latitude - block.lower_bound.latitude
        # take the mid position
        x = block.lower_bound.longitude + (delta_x / 2)
        y = block.lower_bound.latitude + (delta_y / 2)
        return x, y


def load_test_file_in_carla(file: str, carla_server: str = "localhost", carla_port: int = 2000,
                            client_timeout: float = 10.0) -> None:
    """
        Setups the Carla client connection and loads the given file as the simulated world.

        Used for testing via '__main__'.
    Args:
        file (): the file to be loaded as world
        carla_server (): the carla server address
        carla_port (): the carla server pot
        client_timeout (): The timeout for the connection to the carla server in seconds
    """
    client = carla.Client(carla_server, carla_port)
    client.set_timeout(client_timeout)  # seconds
    client.load_world(file)


if __name__ == "__main__":
    """
     The main method for testing purpose.

     Creates the InfrastructureRasterizer and connects to Carla simulation for visualization using the SegmentVisualizer
      based on the given xodr-file
     Args:
        xodr(): the xodr-File to be loaded in InfrastructureRasterizer and Carla simulation
        img(): An Image of the map for visualizing the bounding boxes as figure

    """
    logging.basicConfig(  # filename='/home/malte/block.log', filemode='w',
        format='%(module)s - %(levelname)s: %(message)s', level=logging.INFO)

    parser = ArgumentParser(
        description='Processing of a given XODR file for testing purpose some integers.')
    parser.add_argument('xodr', type=str, help='the opendrive file to be importedr')
    parser.add_argument('-i', '--img', type=str, help='the corresponding bg image for the xodr file to plot onto')
    args = parser.parse_args()

    splitter = InfrastructureRasterizer()
    splitter.analyze_map_from_xodr_file(args.xodr)
    blocks = splitter.blocks
    if blocks:
        if not os.path.exists(args.xodr):
            raise ValueError(f'Given Path "{args.xodr}" is not a valid path on the system.')
        file_name = os.path.basename(args.xodr)
        load_test_file_in_carla(os.path.splitext(file_name)[0])
        segment_visualizer = ScenarioSegmentVisualizer()
        segment_visualizer.visualize_segments_in_carla(blocks, debug_lifespan=0, altitude_offset=15.0,
                                                       draw_block_ids=True,
                                                       line_width=.6)
        debug_lifespan = 150
        segment_visualizer.visualize_segments_as_plot(blocks, args.img)
