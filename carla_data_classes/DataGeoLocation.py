from dataclasses import dataclass

from carla import GeoLocation


@dataclass
class DataGeoLocation:
    """
    Wrapper class for GeoLocation information
    """
    longitude: float
    latitude: float
    altitude: float

    def __init__(self, geo_location: GeoLocation = None, geo_point=None, longitude=None, latitude=None, altitude=None):
        if geo_location is not None:
            self.longitude = geo_location.longitude
            self.latitude = geo_location.latitude
            self.altitude = geo_location.altitude
        elif geo_point is not None:
            self.longitude = float(geo_point.longitude)
            self.latitude = float(geo_point.latitude)
            self.altitude = float(geo_point.altitude)
        elif longitude is not None and latitude is not None and altitude is not None:
            self.longitude = longitude
            self.latitude = latitude
            self.altitude = altitude
        else:
            self.longitude = float(0)
            self.latitude = float(0)
            self.altitude = float(0)

    def to_geolocation(self) -> GeoLocation:
        return GeoLocation(longitude=self.longitude, latitude=self.latitude, altitude=self.altitude)
