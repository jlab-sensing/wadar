import math

def global_path(self, tagDiffLatitude, tagDiffLongitude):
    angle = math.degrees(math.atan2(tagDiffLongitude, tagDiffLatitude))
    return angle