"""Ruskotunturi route frame and front-LiDAR free-space evidence for cooperative SIL."""
from collections import deque
from dataclasses import dataclass, replace
from functools import cached_property, lru_cache
import math

from jarcrac_cooperative_control import A_START, STOP_SPEED
from jarcrac_cooperative_geometry import MODELS, rotate
from jarcrac_lidar_map_assisted import TerrainMeshIndex

# A low-relief part of the committed worksite. Routes and the conflict polygon
# use metres in this local frame. Localisation messages stay in Gazebo's frame.
MAP_ORIGIN = (23.0, 48.0)
MAP_YAW = math.radians(-30.0)
FRONT_SENSOR = (1.68, 0.0, 0.60)
HALF_STEP = math.radians(0.125)
HORIZONTAL_SAMPLES = 721
VERTICAL_SAMPLES = 32
LOCAL_HORIZON = 10.0
GROUND_TOLERANCE = 0.15
BIN_STEP = math.radians(1)
BIN_COUNT = 181
HISTORY_SECONDS = 75.0

# The fixture is generated with this launch area empty. It bridges the
# scanner's minimum range and rear blind area at the first stationary sample.
LAUNCH_X = (A_START[0]-2.90, A_START[0]+2.60)
LAUNCH_Y = (-3.0, 3.0)


@dataclass(frozen=True)
class RouteFrame:
    origin: tuple
    yaw: float

    @cached_property
    def axes(self):
        return math.cos(self.yaw), math.sin(self.yaw)

    def world_vector(self, x, y):
        c, s = self.axes
        return c*x-s*y, s*x+c*y

    def world(self, x, y):
        dx, dy = self.world_vector(x, y)
        return self.origin[0]+dx, self.origin[1]+dy

    def local_vector(self, x, y):
        c, s = self.axes
        return c*x+s*y, -s*x+c*y

    def local(self, x, y):
        return self.local_vector(x-self.origin[0], y-self.origin[1])

    def local_yaw(self, yaw):
        return math.remainder(yaw-self.yaw, math.tau)

    def state(self, state):
        x, y = self.local(state.x, state.y)
        vx, vy = self.local_vector(state.vx, state.vy)
        return replace(state, x=x, y=y, vx=vx, vy=vy, yaw=self.local_yaw(state.yaw))


LOCAL_FRAME = RouteFrame(MAP_ORIGIN, MAP_YAW)
WORLD_FRAME = RouteFrame((0.0, 0.0), 0.0)


def world_xy(x, y):
    return LOCAL_FRAME.world(x, y)


def route_xy(x, y):
    return LOCAL_FRAME.local(x, y)


def route_yaw(world_yaw):
    return LOCAL_FRAME.local_yaw(world_yaw)


def terrain_index(origin=MAP_ORIGIN, radius=42.0):
    mesh = MODELS / "ruskotunturi_map/meshes/ruskotunturi-map-collision.stl"
    return TerrainMeshIndex(mesh, center_x=origin[0], center_y=origin[1],
                            radius_m=radius, cell_m=0.6)


def height_at(terrain, x, y):
    return terrain.height_at(*world_xy(x, y))


def spawn_pose(terrain, footprint, xy, yaw, frame=LOCAL_FRAME):
    """Fit pitch and roll to the local surface, then clear all collision vertices."""
    x, y = frame.world(*xy)
    yaw += frame.yaw
    c, s = math.cos(yaw), math.sin(yaw)

    def support(dx, dy):
        height = terrain.height_at(x+c*dx-s*dy, y+s*dx+c*dy)
        if height is None:
            raise ValueError("Spawn footprint is outside mapped terrain")
        return height

    pitch = -math.atan2(support(1, 0) - support(-1, 0), 2)
    roll = math.atan2(support(0, 0.8) - support(0, -0.8), 1.6)
    z = -math.inf
    for vertex in footprint.vertices:
        dx, dy, dz = rotate(vertex, roll, pitch, yaw)
        height = terrain.height_at(x+dx, y+dy)
        if height is None:
            raise ValueError("Spawn footprint is outside mapped terrain")
        z = max(z, height - dz)
    return x, y, z + 0.018, roll, pitch, yaw


@dataclass(frozen=True)
class FrontScan:
    stamp: float
    x: float
    y: float
    yaw: float
    clearance: tuple
    hazard: tuple
    ground_returns: int = 0
    elevated_returns: int = 0

    def verdict(self, x, y):
        """True for supported free space, False for an obstruction, None for unseen."""
        dx, dy = x-self.x, y-self.y
        distance = math.hypot(dx, dy)
        bearing = math.remainder(math.atan2(dy, dx) - self.yaw, math.tau)
        index = (bearing + math.pi/2) / BIN_STEP
        if distance < 0.30 or not 0 <= index < BIN_COUNT - 1:
            return None
        col = int(index)
        if distance + 0.12 >= min(self.hazard[col], self.hazard[col+1]):
            return False
        if distance + 0.08 <= min(self.clearance[col], self.clearance[col+1]):
            return True
        return None


@lru_cache(maxsize=4)
def beam_vectors(start, step, vertical_start, vertical_step):
    horizontal = [(math.cos(start+i*step), math.sin(start+i*step))
                  for i in range(HORIZONTAL_SAMPLES)]
    vertical = [(math.cos(vertical_start+i*vertical_step), math.sin(vertical_start+i*vertical_step))
                for i in range(VERTICAL_SAMPLES)]
    return tuple((ce*c, ce*s, se) for ce, se in vertical for c, s in horizontal)


def scan_from_message(message, stamp, pose, terrain, frame=LOCAL_FRAME):
    """Reproject the 3-D scan using localisation and ground-only map heights."""
    if (message.count != HORIZONTAL_SAMPLES or message.vertical_count != VERTICAL_SAMPLES
            or len(message.ranges) != HORIZONTAL_SAMPLES * VERTICAL_SAMPLES):
        return None
    start, step = message.angle_min, message.angle_step
    vertical_start, vertical_step = message.vertical_angle_min, message.vertical_angle_step
    if (not all(map(math.isfinite, (start, step, vertical_start, vertical_step,
                                     message.range_min, message.range_max)))
            or not math.isclose((HORIZONTAL_SAMPLES-1)*step, math.pi, abs_tol=0.025)
            or not math.isclose(start, -math.pi/2, abs_tol=0.012)
            or not math.isclose(vertical_start, -math.pi/12, abs_tol=0.015)
            or not math.isclose((VERTICAL_SAMPLES-1)*vertical_step, math.pi/6, abs_tol=0.025)
            or not 0 < message.range_min < message.range_max):
        return None

    ox, oy, oz = rotate(FRONT_SENSOR, pose.roll, pose.pitch, pose.yaw)
    ox, oy, oz = ox+pose.x, oy+pose.y, oz+pose.z
    wx, wy = frame.world(ox, oy)
    ground_at_sensor = terrain.height_at(wx, wy)
    if ground_at_sensor is None or not 0.25 <= oz-ground_at_sensor <= 1.05:
        return None
    c, s = frame.axes
    x_axis, y_axis, z_axis = (rotate(axis, pose.roll, pose.pitch, pose.yaw)
                             for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    free = [0.0] * BIN_COUNT
    hazard = [math.inf] * BIN_COUNT
    ground_returns = elevated_returns = invalid = 0

    for i, ((u, v, w), distance) in enumerate(zip(
            beam_vectors(start, step, vertical_start, vertical_step), message.ranges)):
        if distance == math.inf or LOCAL_HORIZON < distance <= message.range_max:
            continue
        if distance == -math.inf or 0 <= distance < message.range_min:
            bearing = start+(i % HORIZONTAL_SAMPLES)*step
            k = min(BIN_COUNT-1, max(0, round((bearing+math.pi/2)/BIN_STEP)))
            hazard[k] = min(hazard[k], message.range_min)
            continue
        if not message.range_min <= distance <= message.range_max:
            invalid += 1
            continue
        dx = distance * (x_axis[0]*u + y_axis[0]*v + z_axis[0]*w)
        dy = distance * (x_axis[1]*u + y_axis[1]*v + z_axis[1]*w)
        dz = distance * (x_axis[2]*u + y_axis[2]*v + z_axis[2]*w)
        bearing = math.remainder(math.atan2(dy, dx)-pose.yaw, math.tau)
        k = round((bearing+math.pi/2)/BIN_STEP)
        if not 0 <= k < BIN_COUNT:
            continue
        run = math.hypot(dx, dy)
        ground = terrain.height_at(wx+c*dx-s*dy, wy+s*dx+c*dy)
        residual = oz+dz-ground if ground is not None else math.inf
        if abs(residual) <= GROUND_TOLERANCE:
            free[k] = max(free[k], run)
            ground_returns += 1
        else:
            hazard[k] = min(hazard[k], run)
            elevated_returns += residual > GROUND_TOLERANCE and ground is not None
    if invalid > len(message.ranges)*0.05 or sum(d > 0.5 for d in free[55:126]) < 18:
        return None
    return FrontScan(stamp, ox, oy, pose.yaw, tuple(free), tuple(hazard), ground_returns, elevated_returns)


class ObservedSpace:
    """Keep recent ground-supported views of the course's static obstacles."""
    def __init__(self, launch_frame=WORLD_FRAME, start=A_START, launch_x=LAUNCH_X, launch_y=LAUNCH_Y):
        self.frames = deque(maxlen=128)
        self.current = None
        self.launch_clear = False
        self.launch_frame, self.start = launch_frame, start
        self.launch_x, self.launch_y = launch_x, launch_y

    def update(self, scan, state):
        if self.current is None and not self.frames:
            local = self.launch_frame.state(state)
            self.launch_clear = (math.dist((local.x, local.y), self.start) <= 0.25
                                 and local.speed <= STOP_SPEED and abs(local.omega) <= STOP_SPEED
                                 and abs(local.yaw) <= 0.15)
        if self.current is not None and scan.stamp <= self.current.stamp:
            return self
        if self.current is not None:
            previous = self.frames[0] if self.frames else None
            if (previous is None or self.current.stamp-previous.stamp > 2.0
                    or math.dist((self.current.x, self.current.y), (previous.x, previous.y)) > 0.18
                    or abs(math.remainder(self.current.yaw-previous.yaw, math.tau)) > 0.08):
                self.frames.appendleft(self.current)
        self.current = scan
        while self.frames and scan.stamp-self.frames[-1].stamp > HISTORY_SECONDS:
            self.frames.pop()
        return self

    @property
    def stamp(self):
        return self.current.stamp

    def covers(self, x, y):
        current = self.current.verdict(x, y)
        if current is not None:
            return current
        for frame in self.frames:
            verdict = frame.verdict(x, y)
            if verdict is not None:
                return verdict
        lx, ly = self.launch_frame.local(x, y)
        return (self.launch_clear and self.launch_x[0] <= lx <= self.launch_x[1]
                and self.launch_y[0] <= ly <= self.launch_y[1])
