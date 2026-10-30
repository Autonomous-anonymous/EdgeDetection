"""Startup-readiness diagnostics for the Ruskotunturi R2100 HIL path."""

from __future__ import annotations

from dataclasses import dataclass


BRAKE_REASONS = {
    0: "none",
    1: "emergency_distance",
    2: "ttc_threshold",
    3: "sensor_timeout",
}


@dataclass(frozen=True)
class StartupReadiness:
    r2100_host: str
    pose_fresh: bool
    odometry_fresh: bool
    brake_reply_fresh: bool
    brake_active: bool
    brake_reason: int
    fault: str | None = None
    lidar_error: str | None = None

    @property
    def brake_reason_text(self) -> str:
        return BRAKE_REASONS.get(
            self.brake_reason,
            f"unknown({self.brake_reason})",
        )

    def format_report(self) -> str:
        status = lambda ok: "PASS" if ok else "FAIL"

        return "\n".join([
            "HIL startup readiness",
            "",
            f"R2100 host           : {self.r2100_host}",
            f"[{status(self.pose_fresh)}] Gazebo pose",
            f"[{status(self.odometry_fresh)}] Gazebo odometry",
            f"[{status(self.brake_reply_fresh)}] R2100 brake reply",
            (
                "Brake state          : "
                f"{'active' if self.brake_active else 'released'} "
                f"({self.brake_reason_text})"
            ),
            f"Runtime fault        : {self.fault or 'none'}",
            f"LiDAR sender error   : {self.lidar_error or 'none'}",
        ])

    def failure_reason(self) -> str | None:
        if self.lidar_error:
            return "startup_lidar_send_error"

        if self.fault == "udp_error":
            return "startup_udp_error"

        if not self.pose_fresh and not self.odometry_fresh:
            return "startup_gazebo_not_ready"

        if not self.pose_fresh:
            return "startup_pose_not_ready"

        if not self.odometry_fresh:
            return "startup_odometry_not_ready"

        if not self.brake_reply_fresh:
            return "startup_brake_reply_missing"

        if self.brake_active:
            if self.brake_reason == 3:
                return "startup_r2100_sensor_timeout"

            return "startup_brake_active"

        if self.fault:
            return f"startup_{self.fault}"

        return None

    def ready(self) -> bool:
        return self.failure_reason() is None
