import math

def global_path(self_latitude, self_longitude, self_heading, tag_latitude, tag_longitude):
    """
    Calculate the relative angle from the current heading to the target location.

    Args:
        self_latitude (float): Current latitude.
        self_longitude (float): Current longitude.
        self_heading (float): Current heading in radians.
        tagLatitude (float): Target latitude.
        tagLongitude (float): Target longitude.

    Returns:
        float: Relative angle to the target location in degrees.
    """
    lat1 = math.radians(self_latitude)
    lon1 = math.radians(self_longitude)
    lat2 = math.radians(tag_latitude)
    lon2 = math.radians(tag_longitude)

    bearingAngle = math.atan2(math.sin(lon2 - lon1) * math.cos(lat2), math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
    bearingAngle = math.degrees(bearingAngle)
    bearingAngle = (bearingAngle + 360) % 360

    self_heading_degrees = math.degrees(self_heading)
    relativeAngle = bearingAngle - self_heading_degrees

    relativeAngle = (relativeAngle + 360) % 360
    return relativeAngle