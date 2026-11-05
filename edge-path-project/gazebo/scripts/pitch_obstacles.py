"""Sensor-only obstacle observations for the short pitch, without Gazebo imports.

Stationary means an unchanged visible surface while OUR vehicle is measured at
rest. It is a bounded observation, not an object identity or an intent estimate.
"""

import math

from final_sil_v6_checks import point_clearance, scan_valid

STATIONARY_SECONDS = 2.0
MIN_STATIONARY_SCANS = 15
PROFILE_TOLERANCE_M = .15
MAX_SCAN_GAP_S = .30
TTC_BRAKE_S = 2.0
TTC_CAUTION_S = 3.0
MIN_CLEARANCE_M = 1.1


class ObstacleObservation:
    def __init__(self):
        self.stamp = None
        self.serial = 0
        self.received = 0.0
        self.valid = False
        self.front = self.clearance = self.ttc = math.inf
        self.speed = 0.0
        self.holding = False
        self.clear_scans = 0
        self.anchor = None
        self.stationary_scans = 0
        self.stationary_seconds = 0.0
        self.brake_count = 0
        self.braking = False
        self.minimum_ttc = math.inf
        self.speed_limited_commands = 0

    def reset_observation(self):
        self.anchor = None
        self.stationary_scans = 0
        self.stationary_seconds = 0.0

    def hold(self):
        self.holding = True
        self.clear_scans = 0
        self.reset_observation()

    def release(self):
        self.holding = False
        self.reset_observation()

    @property
    def stationary(self):
        return (self.valid and self.holding and self.stationary_seconds >= STATIONARY_SECONDS
                and self.stationary_scans >= MIN_STATIONARY_SCANS)

    def update(self, scan, speed, pose, *, odometry_fresh, received):
        header = getattr(scan, "header", None)
        if header is None or not header.HasField("stamp"):
            self.valid = False
            self.reset_observation()
            return
        stamp = header.stamp.sec + header.stamp.nsec * 1e-9
        if self.stamp is not None and stamp <= self.stamp:
            # Cached/replayed scans never add observation time or clear votes.
            return
        gap = math.inf if self.stamp is None else stamp - self.stamp
        self.stamp = stamp
        self.received = received
        self.serial += 1
        self.valid = scan_valid(scan)
        self.speed = speed if odometry_fresh else math.inf
        self.front = self.clearance = self.ttc = math.inf
        if not self.valid:
            self.clear_scans = 0
            self.reset_observation()
            return
        points = []
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_step
            if abs(angle) > math.radians(35) or not math.isfinite(distance):
                continue
            if not scan.range_min <= distance <= scan.range_max:
                continue
            if abs(angle) <= math.radians(20):
                self.front = min(self.front, distance)
                self.clearance = min(self.clearance, point_clearance(distance, angle))
            points.append((distance, distance * math.cos(angle), distance * math.sin(angle)))
        # Same distance/measured-speed TTC approximation as the R2100 baseline,
        # using bumper clearance. This is not relative-velocity object tracking.
        if odometry_fresh and speed >= .1:
            self.ttc = self.clearance / speed
            self.minimum_ttc = min(self.minimum_ttc, self.ttc)
        if gap > MAX_SCAN_GAP_S:
            self.clear_scans = 0
        self.clear_scans = self.clear_scans + 1 if self.front >= 11.5 else 0
        if (not self.holding or not odometry_fresh or speed > .05
                or not math.isfinite(self.front) or self.front > 11.5):
            self.reset_observation()
            return
        # Follow the extent of the near surface, including lateral motion.
        # A single constant nearest range cannot establish a stationary object.
        near = [(x, y) for d, x, y in points if d <= self.front + 1.0]
        if len(near) < 5:
            self.reset_observation()
            return
        bounds = (min(x for x, _ in near), max(x for x, _ in near),
                  min(y for _, y in near), max(y for _, y in near))
        unchanged = False
        if self.anchor is not None and gap <= MAX_SCAN_GAP_S:
            _, old_bounds, old_pose = self.anchor
            unchanged = (max(abs(a-b) for a, b in zip(bounds, old_bounds)) <= PROFILE_TOLERANCE_M
                         and math.hypot(pose[0]-old_pose[0], pose[1]-old_pose[1]) <= .08
                         and abs(math.atan2(math.sin(pose[2]-old_pose[2]), math.cos(pose[2]-old_pose[2]))) <= .02)
        if not unchanged:
            self.anchor = (stamp, bounds, pose)
            self.stationary_scans = 1
            self.stationary_seconds = 0.0
        else:
            self.stationary_scans += 1
            self.stationary_seconds = stamp - self.anchor[0]

    def command(self, command):
        """Brake on measured risk; also limit requested speed before risk grows."""
        speed, turn = command
        brake = not self.valid or self.clearance <= MIN_CLEARANCE_M or self.ttc <= TTC_BRAKE_S
        if brake and not self.braking:
            self.brake_count += 1
        self.braking = brake
        if brake or self.holding:
            return 0.0, 0.0
        if speed <= 0:
            return command
        safe_speed = min(speed, self.clearance / TTC_CAUTION_S)
        if safe_speed < speed:
            self.speed_limited_commands += 1
        return safe_speed, turn * safe_speed / speed

    def evidence(self):
        return dict(stationary_seconds=round(self.stationary_seconds, 3),
                    stationary_scans=self.stationary_scans,
                    front_m=None if not math.isfinite(self.front) else round(self.front, 3),
                    clearance_m=None if not math.isfinite(self.clearance) else round(self.clearance, 3),
                    ttc_s=None if not math.isfinite(self.ttc) else round(self.ttc, 3),
                    minimum_ttc_s=None if not math.isfinite(self.minimum_ttc) else round(self.minimum_ttc, 3),
                    speed_limited_commands=self.speed_limited_commands,
                    ttc_or_clearance_brakes=self.brake_count)
