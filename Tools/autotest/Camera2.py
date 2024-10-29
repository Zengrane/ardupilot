from typing import List, Tuple, Union
import cuav_util_modified
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon, Point
import pymap3d
import math


def auto_str(cls):
    def __str__(self):
        return '%s(%s)' % (
            type(self).__name__,
            ', '.join('%s=%s' % item for item in vars(self).items())
        )

    cls.__str__ = __str__
    cls.__repr__ = __str__
    return cls


def constrain_pitch(pitch: float) -> float:
    """
    Constrain pitch to ±90 degrees.
    :param pitch: Current pitch value
    :return: Constrained pitch value
    """
    return max(min(pitch, 90), -90)


def constrain_yaw(yaw: float) -> float:
    """
    Constrain yaw to 0-360 degrees (wrap around).
    :param yaw: Current yaw value
    :return: Constrained yaw value (0 to 360 degrees)
    """
    return yaw % 360


def constrain_roll(roll: float) -> float:
    """
    Constrain roll to -45 to 45 degrees.
    :param roll: Current roll value
    :return: Constrained roll value
    """
    return max(min(roll, 45), -45)


@auto_str
class Attitude:
    def __init__(self, yaw_deg: float, pitch_deg: float, roll_deg: float):
        self.yaw_deg = constrain_yaw(yaw_deg)
        self.pitch_deg = constrain_pitch(pitch_deg)
        self.roll_deg = constrain_roll(roll_deg)

    @property
    def pry_deg(self) -> Tuple[float, float, float]:
        return self.pitch_deg, self.roll_deg, self.yaw_deg


@auto_str
class PositionLLA:
    def __init__(self, lat: float, lon: float, alt: Union[None, float]):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    @property
    def lla(self) -> Tuple[float, float, float]:
        return self.lat, self.lon, self.alt

    @property
    def ll(self) -> Tuple[float, float]:
        return self.lat, self.lon


@auto_str
class Camera:
    def __init__(self, res_px: Tuple[int, int], lens_m: float, sensor_size_m: float,
                 mounting_yaw_deg: float, mounting_pitch_deg: float, mounting_roll_deg: float):
        self.res_px = res_px
        self._num_pixels = self.res_px[0] * self.res_px[1]
        self.lens_m = lens_m
        self.sensor_size_m = sensor_size_m
        self.mounting_yaw_deg = mounting_yaw_deg
        self.mounting_pitch_deg = mounting_pitch_deg
        self.mounting_roll_deg = mounting_roll_deg

    @property
    def num_pixels(self) -> int:
        return self._num_pixels

    @staticmethod
    def basler_camera(mounting_yaw_deg: float, mounting_pitch_deg: float, mounting_roll_deg: float) -> 'Camera':
        return Camera((4096, 3000), 0.008, 0.0141, mounting_yaw_deg, mounting_pitch_deg, mounting_roll_deg)


def calculate_pitch_to_target(uav_position: PositionLLA, target_position: PositionLLA) -> float:
    """
    Calculate the pitch angle required to keep the target within the camera's FOV.
    :param uav_position: UAV position as a PositionLLA object
    :param target_position: Target position as a PositionLLA object
    :return: Updated pitch angle in degrees
    """
    # Calculate distance to the target in the horizontal plane
    distance = pymap3d.geodetic2enu(target_position.lat, target_position.lon, 0, uav_position.lat, uav_position.lon, 0)
    distance_horiz = math.sqrt(distance[0] ** 2 + distance[1] ** 2)  # Horizontal distance

    # Calculate the required pitch angle (in degrees) to keep the target in view
    delta_alt = target_position.alt - uav_position.alt
    pitch_angle = math.degrees(math.atan2(delta_alt, distance_horiz))

    return pitch_angle


def camera_perimeter(camera: Camera,
                     position: PositionLLA,
                     attitude: Attitude,
                     range_m: float) -> Union[None, List[Tuple[float, float,]]]:
    """
    Given the position and attitude of the aircraft, calculate if any targets are detectable
    :param camera: camera object representing the physical hardware
    :param position: Position of UAS
    :param attitude: Attitude of UAS
    :param range_m: Range of the camera in meters
    :return: A list of camera corners projected onto the ground
    """
    camera_params = cuav_util_modified.CameraParams(camera.lens_m, camera.sensor_size_m, *camera.res_px)  # CUAV object

    # Calculate the pixel position offsets of the camera corners
    pitch, roll, yaw = attitude.pry_deg
    pixel_positions = [cuav_util_modified.pixel_coordinates(px[0], px[1],
                                                            *position.lla,
                                                            pitch + camera.mounting_pitch_deg,
                                                            roll + camera.mounting_roll_deg,
                                                            yaw + camera.mounting_yaw_deg,
                                                            camera_params) for px in
                       [(0, 0), (camera_params.xresolution, 0), (camera_params.xresolution, camera_params.yresolution),
                        (0, camera_params.yresolution)]]

    if any(pixel_position is None for pixel_position in pixel_positions):
        # At least one of the pixels is not on the ground so it doesn't make sense to try to draw the polygon
        return None

    trapezoid_points = []
    for pp in pixel_positions:
        x, y, _ = pymap3d.geodetic2enu(pp[0], pp[1], 0, *position.ll, 0)
        trapezoid_points.append(Point(x, y))

    camera_polygon = Polygon(trapezoid_points)
    camera_range = Point(0, 0).buffer(range_m)
    intersection = camera_polygon.intersection(camera_range)

    try:
        perim_points = [pymap3d.enu2geodetic(e, n, 0, position.lat, position.lon, 0) for e, n in
                        zip(*intersection.exterior.xy)]
        return [(pp[0], pp[1]) for pp in perim_points]
    except:
        return None


if __name__ == "__main__":
    init_lat = -32.0
    init_lon = 138.0
    cam_pitch = 49.2  # cam_pitch = 0 (looking straight down)
    cam_yaw = 10.
    cam_roll = 0.
    r = 0
    p = 0
    y = 0
    rangee = 500
    pos = PositionLLA(init_lat, init_lon, 100)

    # Simulating a moving target
    target_lat = -31.999
    target_lon = 138.001
    target_alt = 90
    target_pos = PositionLLA(target_lat, target_lon, target_alt)

    att = Attitude(y, p, r)

    # Update the camera's pitch to keep the target within the FOV
    dynamic_pitch = calculate_pitch_to_target(pos, target_pos)
    att.pitch_deg = constrain_pitch(dynamic_pitch)  # Apply the pitch constraint

    perim_points = camera_perimeter(Camera.basler_camera(cam_yaw, cam_pitch, cam_roll), pos, att, rangee)
    if perim_points is None:
        print("Camera corners do not intersect ground")
    else:
        xs = []
        ys = []
        [(xs.append(pp[1]), ys.append(pp[0])) for pp in perim_points]
        plt.plot(xs, ys, '-r')

        plt.title(
            f"Range {rangee}m, Mount Pitch {cam_pitch}, Mount Yaw {cam_yaw}, Mount Roll {cam_roll}, RPY {r, p, y}deg")
        plt.xlabel("lon")
        plt.ylabel("lat")
        plt.plot(init_lon, init_lat, "ob")
        plt.show()
