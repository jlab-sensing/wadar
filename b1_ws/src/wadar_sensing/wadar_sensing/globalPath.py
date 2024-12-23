import math

def global_path(self_latitude, self_longitude, self_heading, tagLatitude, tagLongitude):
    lat1 = math.radians(self_latitude)
    lon1 = math.radians(self_longitude)
    lat2 = math.radians(tagLatitude)
    lon2 = math.radians(tagLongitude)

    bearingAngle = math.atan2(math.sin(lon2 - lon1) * math.cos(lat2), math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
    bearingAngle = math.degrees(bearingAngle)
    bearingAngle = (bearingAngle + 360) % 360
    relativeAngle = bearingAngle - self_heading

    return relativeAngle