from typing import List, Tuple, Union
import cuav_util_modified
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon, Point
import pymap3d


def auto_str(cls):
    def __str__(self):
        return '%s(%s)' % (
            type(self).__name__,
            ', '.join('%s=%s' % item for item in vars(self).items())
        )

    cls.__str__ = __str__
    cls.__repr__ = __str__
    return cls


@auto_str
class Attitude:
    def __init__(self, yaw_deg: float, pitch_deg: float, roll_deg: float):
        self.yaw_deg = yaw_deg
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg

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


def camera_perimeter(camera: Camera,
                     position: PositionLLA,
                     attitude: Attitude,
                     range_m: float) -> Union[None, List[Tuple[float, float,]]]:
    """
    Given the position and attitude of the aircraft, calculate if any targets are detectable
    :param camera: camera object representing the physical hardware
    :param position: Position of UAS
    :param attitude: Attitude of UAS
    :return: A list camera corners projected onto the ground
    """
    camera_params = cuav_util_modified.CameraParams(camera.lens_m, camera.sensor_size_m, *camera.res_px)  # CUAV object

    # calculate the pixel position offsets of the camera corners
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
        # at least one of the pixels is not on the ground so it doesn't make sense to try to draw the polygon
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
    cam_pitch = 49.2  # cam_pitch = 0 (looking down)
    cam_yaw = -90.
    cam_roll = 0.
    r = 0
    p = 0
    y = 0
    rangee = 500
    pos = PositionLLA(init_lat, init_lon, 100)
    att = Attitude(y, p, r)

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
