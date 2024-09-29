import numpy as np
from shapely.geometry import Polygon, Point, LineString
from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation
from abc import ABC, abstractmethod


@dataclass
class CameraView_Abstract(ABC):
    @abstractmethod
    def at(self, x, y, z, roll, pitch, yaw, degree=False): pass

    def distance(self, x0, y0, z0, x1, y1, z1):
        return np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2 + (z0 - z1) ** 2)

    def distance_xy(self, x0, y0, x1, y1):
        return np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)


@dataclass
class CameraView_FOV(CameraView_Abstract):
    pitch: float
    vfov: float
    hfov: float
    max_2d_range: float = field(default=np.inf)
    include_arc: bool = field(default=False)

    def __post_init__(self):
        self.cam_rot = Rotation.from_euler('xyz', [[0, self.pitch - self.vfov / 2, -self.hfov / 2],  # bottom-left
            [0, self.pitch - self.vfov / 2, +self.hfov / 2],  # bottom-right
            [0, self.pitch + self.vfov / 2, +self.hfov / 2],  # top-right
            [0, self.pitch + self.vfov / 2, -self.hfov / 2]  # top-left
        ])

    def at(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, degree: bool = False) -> Polygon:
        vec = self._gnd_prj_vec(x, y, z, roll, pitch, yaw, degree)
        vec_sort = self._topo_sort_basic(vec)
        p = Polygon(vec_sort[:, :2])
        assert p.is_valid, NotImplementedError('Something wrong with topology')
        if self.max_2d_range < np.inf:
            p = p.intersection(Point([x, y]).buffer(self.max_2d_range))
        return p

    def _gnd_prj_vec(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float,
                     degree: bool = field(default=False)) -> np.ndarray:
        unit_cam = self._get_unit_vec_cam(roll, pitch, yaw, degree)
        vec = self._get_fov_unit_vec(roll, pitch, yaw, degree)
        vec_valid = self._get_valid_grounded_vecs(z, vec)

        if len(vec_valid) < 4:
            assert not np.isinf(self.max_2d_range), NotImplementedError("FOV beyond horizon")
            # Find the planes (and its coresponding line)
            vidx = [0, 1, 2, 3, 0]
            line_coef = [self._vecs2gndprjline(vec[vidx[i]], vec[vidx[i + 1]], 0, 0, z) for i in
                         range(0, len(vidx) - 1)]
            # Find the line with only one valid vec point
            ln_ext = [ln_c for ln_c in line_coef if
                sum(abs(self._eval_2Dline(vec_valid[:, 0], vec_valid[:, 1], *ln_c)) < 1e-9) == 1]
            # Choose a point on each line and add it to the valid vec
            vec_new = vec_valid
            for (a, b, c) in ln_ext:
                vec_temp = self._get_circle_line_intersect(a, b, c, x, y)
                if sum(vec_temp[0] * unit_cam[0:2]).T > 0:
                    vec_temp = vec_temp[0]
                else:
                    vec_temp = vec_temp[1]
                vec_new = np.array(list(vec_new) + [np.append(vec_temp, -z)])
        else:
            vec_new = vec_valid
        assert not np.all(np.linalg.norm(vec_new, axis=1) > self.max_2d_range), NotImplementedError("FOV out of range")

        vec_new[:, 0] += x
        vec_new[:, 1] += y
        return vec_new

    def _get_unit_vec_cam(self, roll: float, pitch: float, yaw: float, degree: bool = False) -> np.ndarray:
        if degree:
            roll, pitch, yaw = np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)
        yaw *= -1
        cam_rot = Rotation.from_euler('xyz', [0, self.pitch, 0])
        drone_rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
        unit_vec = [1, 0, 0] @ cam_rot.as_matrix() @ drone_rot.as_matrix()
        return unit_vec

    def _get_fov_unit_vec(self, roll: float, pitch: float, yaw: float, degree: bool = False) -> np.ndarray:
        if degree:
            roll, pitch, yaw = np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)
        yaw *= -1
        drone_rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
        vec = [1, 0, 0] @ self.cam_rot.as_matrix() @ drone_rot.as_matrix()
        return vec

    def _get_valid_grounded_vecs(self, z: float, vec: np.ndarray) -> np.ndarray:
        assert not np.all(vec[:, 2] >= 0.0), NotImplementedError("FOV looking at the sky exclusively!")
        horizon_flag = vec[:, 2] < 0.0
        vec_prj = vec * (-z / np.atleast_2d(vec[:, 2]).T)
        return vec_prj[horizon_flag]

    def _vecs2gndprjline(self, unit_v1: np.ndarray, unit_v2: np.ndarray, x0: float, y0: float, z0: float) -> float:
        abc = np.cross(unit_v1, unit_v2)
        d = -np.dot(abc, np.array([x0, y0, z0]))
        if abc[1] == 0:
            abc[1] = 1e-15
        return abc[0], abc[1], d

    def _topo_sort_basic(self, vec: np.ndarray) -> np.ndarray:
        cntr_pt = np.mean(vec, axis=0)
        diff_pt = vec[:, :2] - cntr_pt[:2]
        sort_idx = np.argsort(np.rad2deg(np.arctan2(diff_pt[:, 1], diff_pt[:, 0])) % 360)
        return vec[sort_idx]

    def _get_circle_line_intersect(self, a: float, b: float, c: float, x: float, y: float) -> np.ndarray:
        (A, B, C) = (1 + (a / b) ** 2, 2 * ((a / b) * (c / b + y) - x),
                     x ** 2 + (c / b + y) ** 2 - ((1e10) * self.max_2d_range) ** 2)
        xn = np.array([(-B + np.sqrt(B ** 2 - 4 * A * C)) / (2 * A), (-B - np.sqrt(B ** 2 - 4 * A * C)) / (2 * A)])
        yn = -(a * xn + c) / b
        return np.array([xn, yn]).T

    def _eval_2Dline(self, x: float, y: float, a: float, b: float, c: float) -> float:
        return a * x + b * y + c


@dataclass
class CameraView(CameraView_FOV):
    location_error: float = 35.0

    def get_location_error(self, x0, y0, z0, roll0, pitch0, yaw0, x1, y1, z1, roll1, pitch1, yaw1, degree=False):
        return self.location_error


@dataclass
class CameraView_Trial(CameraView):
    pitch: float = field(default=np.deg2rad(-49.2))
    vfov: float = field(default=np.deg2rad(50.0))
    hfov: float = field(default=np.deg2rad(60.0))
    max_2d_range: float = field(default=300.0)
    location_error: float = 35.0

    def at(self, x, y, z, roll, pitch, yaw):
        try:
            view = CameraView.at(self, x, y, z, roll, pitch, yaw)
        except:
            view = Polygon()
        return view


@dataclass
class CameraView_Gimbal_Trial(CameraView_Trial):
    def at(self, x, y, z, roll, pitch, yaw):
        try:
            view = CameraView.at(self, x, y, z, 0, 0, yaw)
        except:
            view = Polygon()
        return view


if __name__ == "__main__":
    from matplotlib import pyplot as plt

    cam_pitch = -np.deg2rad(45)
    max_2d_range = 600

    uas_x, uas_y, uas_z = [0, 0, 200.]
    roll, pitch, yaw = np.deg2rad([0, 0, 0])

    vfov = np.deg2rad(50.0)
    hfov = np.deg2rad(40.0)

    fig, ax = plt.subplots(num=1, clear=True)
    ax.axis(np.array([-1, 1, -1, 1]) * 500)
    ax.axis('square')

    cam = CameraView_Gimbal_Trial(cam_pitch, vfov, hfov)
    p_cam = cam.at(uas_x, uas_y, uas_z, roll, pitch, yaw)

    ax.plot(uas_x, uas_y, 'g*')
    ax.plot(*p_cam.exterior.xy, 'g-')
    plt.show()

    # plt.savefig("gimbal-footprint.png")