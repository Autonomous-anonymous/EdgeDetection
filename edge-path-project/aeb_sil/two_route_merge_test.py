import carla
import csv
import json
import math
import time

from datetime import datetime
from pathlib import Path

import zmq


# =========================================================
# CONFIGURATION
# =========================================================

FIXED_DELTA_SECONDS = 0.05

ROUTE_STEP_M = 2.0

BRANCH_DISTANCE_M = 45.0
COMMON_ROUTE_DISTANCE_M = 140.0

MERGE_PROBE_DISTANCES_M = [
    6.0,
    10.0,
    14.0,
    20.0,
]

LOOKAHEAD_POINTS = 4


# =========================================================
# VEHICLES
# =========================================================

VEHICLE_A_SPEED_KMH = 30.0
VEHICLE_B_SPEED_KMH = 18.0


# =========================================================
# PRE-MERGE COORDINATION
# =========================================================

MERGE_HEADWAY_SECONDS = 4.0

MERGE_GATE_DISTANCE_M = 20.0
MERGE_RELEASE_LEAD_M = 12.0


# =========================================================
# POST-MERGE SPEED COORDINATION
# =========================================================

FOLLOW_FREE_GAP_M = 22.0
FOLLOW_MATCH_GAP_M = 14.0
FOLLOW_CLOSE_GAP_M = 9.0


# =========================================================
# LOADING ZONE COORDINATION
# =========================================================

# Jos edellä ajava B on jo pysähtynyt loading zoneen,
# A saa jatkaa hitaasti omaan tavoitepisteeseensä.
LOADING_ZONE_CRAWL_SPEED_KMH = 3.0

# Jos B on pysähtynyt, A ei enää ryömi tätä
# pienempään bumper-to-bumper-väliin.
STOPPED_LEAD_MIN_GAP_M = 10.0


# =========================================================
# SAFETY
# =========================================================

SAFETY_RELEASE_GAP_M = 10.0
SAFETY_MARGIN_M = 1.0

STOP_SPEED_KMH = 0.5


# =========================================================
# LOADING ZONE
# =========================================================

VEHICLE_A_GOAL_OFFSET_M = 18.0

GOAL_TOLERANCE_M = 4.0


# =========================================================
# TEST
# =========================================================

TEST_TIMEOUT_SECONDS = 90.0


# =========================================================
# RESULTS
# =========================================================

PROJECT_ROOT = (
    Path(__file__)
    .resolve()
    .parent
    .parent
)

RESULTS_DIR = (
    PROJECT_ROOT
    / "results"
)

RESULTS_FILE = (
    RESULTS_DIR
    / "two_route_merge_results.csv"
)


# =========================================================
# ZEROMQ
# =========================================================

SENSOR_ADDRESS = (
    "tcp://127.0.0.1:5555"
)

CONTROL_ADDRESS = (
    "tcp://127.0.0.1:5556"
)


# =========================================================
# GENERAL HELPERS
# =========================================================

def clamp(
    value,
    minimum,
    maximum
):

    return max(
        minimum,
        min(
            maximum,
            value
        )
    )


def normalize_angle(
    angle_deg
):

    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


def yaw_difference(
    yaw_a,
    yaw_b
):

    return abs(
        normalize_angle(
            yaw_b - yaw_a
        )
    )


def get_speed_kmh(
    vehicle
):

    velocity = (
        vehicle.get_velocity()
    )

    speed_ms = math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )

    return (
        speed_ms
        * 3.6
    )


def waypoint_key(
    waypoint
):

    return (
        waypoint.road_id,
        waypoint.lane_id,
        round(
            waypoint.s,
            1
        )
    )


def make_spawn_transform(
    waypoint
):

    transform = (
        waypoint.transform
    )

    return carla.Transform(

        carla.Location(
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z + 0.3
        ),

        transform.rotation
    )


# =========================================================
# ROUTE DISTANCE
# =========================================================

def build_cumulative_distances(
    route
):

    cumulative = [
        0.0
    ]

    total = 0.0

    for index in range(
        1,
        len(route)
    ):

        previous_location = (
            route[index - 1]
            .transform
            .location
        )

        current_location = (
            route[index]
            .transform
            .location
        )

        total += (
            previous_location.distance(
                current_location
            )
        )

        cumulative.append(
            total
        )

    return cumulative


def get_route_length(
    route
):

    cumulative = (
        build_cumulative_distances(
            route
        )
    )

    return cumulative[-1]


def find_index_before_end(
    route,
    distance_before_end_m
):

    cumulative = (
        build_cumulative_distances(
            route
        )
    )

    total_length = (
        cumulative[-1]
    )

    target_distance = max(
        0.0,
        total_length
        - distance_before_end_m
    )

    best_index = 0
    best_error = float(
        "inf"
    )

    for index, distance in enumerate(
        cumulative
    ):

        error = abs(
            distance
            - target_distance
        )

        if (
            error
            < best_error
        ):

            best_error = (
                error
            )

            best_index = (
                index
            )

    return best_index


# =========================================================
# ROUTE BUILDING
# =========================================================

def choose_straightest(
    current_waypoint,
    candidates
):

    current_yaw = (
        current_waypoint
        .transform
        .rotation
        .yaw
    )

    return min(

        candidates,

        key=lambda waypoint:
            yaw_difference(

                current_yaw,

                waypoint
                .transform
                .rotation
                .yaw
            )
    )


def extend_backward(
    start_waypoint,
    distance_m
):

    route_reverse = [
        start_waypoint
    ]

    current = (
        start_waypoint
    )

    travelled = 0.0

    while (
        travelled
        < distance_m
    ):

        candidates = (
            current.previous(
                ROUTE_STEP_M
            )
        )

        if not candidates:
            break

        same_lane = [

            waypoint

            for waypoint
            in candidates

            if (
                waypoint.road_id
                == current.road_id

                and

                waypoint.lane_id
                == current.lane_id
            )
        ]

        if same_lane:

            previous_waypoint = (
                same_lane[0]
            )

        else:

            previous_waypoint = (
                choose_straightest(
                    current,
                    candidates
                )
            )

        segment_distance = (
            current
            .transform
            .location
            .distance(
                previous_waypoint
                .transform
                .location
            )
        )

        travelled += (
            segment_distance
        )

        route_reverse.append(
            previous_waypoint
        )

        current = (
            previous_waypoint
        )

    route_reverse.reverse()

    return route_reverse


def extend_forward(
    start_waypoint,
    distance_m
):

    route = [
        start_waypoint
    ]

    current = (
        start_waypoint
    )

    travelled = 0.0

    while (
        travelled
        < distance_m
    ):

        candidates = (
            current.next(
                ROUTE_STEP_M
            )
        )

        if not candidates:
            break

        same_lane = [

            waypoint

            for waypoint
            in candidates

            if (
                waypoint.road_id
                == current.road_id

                and

                waypoint.lane_id
                == current.lane_id
            )
        ]

        if same_lane:

            next_waypoint = (
                same_lane[0]
            )

        else:

            next_waypoint = (
                choose_straightest(
                    current,
                    candidates
                )
            )

        segment_distance = (
            current
            .transform
            .location
            .distance(
                next_waypoint
                .transform
                .location
            )
        )

        travelled += (
            segment_distance
        )

        route.append(
            next_waypoint
        )

        current = (
            next_waypoint
        )

    return route


# =========================================================
# PATH SEARCH TO MERGE
# =========================================================

def find_forward_path(
    start_waypoint,
    target_waypoint,
    max_steps=60,
    beam_width=30
):

    target_location = (
        target_waypoint
        .transform
        .location
    )

    frontier = [
        [
            start_waypoint
        ]
    ]

    for _ in range(
        max_steps
    ):

        next_frontier = []

        for path in frontier:

            current = (
                path[-1]
            )

            distance_to_target = (
                current
                .transform
                .location
                .distance(
                    target_location
                )
            )

            if (
                distance_to_target
                <= 2.5
            ):

                return (
                    path
                    + [
                        target_waypoint
                    ]
                )

            candidates = (
                current.next(
                    ROUTE_STEP_M
                )
            )

            existing_keys = {

                waypoint_key(
                    waypoint
                )

                for waypoint
                in path
            }

            for candidate in candidates:

                candidate_key = (
                    waypoint_key(
                        candidate
                    )
                )

                if (
                    candidate_key
                    in existing_keys
                ):

                    continue

                next_frontier.append(

                    path
                    + [
                        candidate
                    ]
                )

        if not next_frontier:
            break

        def score(
            path
        ):

            waypoint = (
                path[-1]
            )

            distance_score = (
                waypoint
                .transform
                .location
                .distance(
                    target_location
                )
            )

            yaw_score = (
                yaw_difference(

                    waypoint
                    .transform
                    .rotation
                    .yaw,

                    target_waypoint
                    .transform
                    .rotation
                    .yaw
                )
            )

            return (
                distance_score
                + 0.02
                * yaw_score
            )

        next_frontier.sort(
            key=score
        )

        frontier = (
            next_frontier[
                :beam_width
            ]
        )

    return None


# =========================================================
# FIND MERGE SCENARIO
# =========================================================

def find_merge_scenario(
    world
):

    carla_map = (
        world.get_map()
    )

    waypoints = (
        carla_map.generate_waypoints(
            4.0
        )
    )

    print(
        "Etsitään yhteistä waypointia, "
        "johon tulee kaksi eri reittiä..."
    )

    tested_candidates = 0

    for common_waypoint in waypoints:

        if (
            common_waypoint.is_junction
        ):

            continue

        common_forward = (
            extend_forward(
                common_waypoint,
                COMMON_ROUTE_DISTANCE_M
            )
        )

        if (
            get_route_length(
                common_forward
            )
            < 100.0
        ):

            continue

        for probe_distance in (
            MERGE_PROBE_DISTANCES_M
        ):

            predecessors = (
                common_waypoint.previous(
                    probe_distance
                )
            )

            unique_predecessors = {}

            for predecessor in predecessors:

                if (
                    predecessor.lane_type
                    != carla.LaneType.Driving
                ):

                    continue

                unique_predecessors[
                    waypoint_key(
                        predecessor
                    )
                ] = predecessor

            predecessors = list(
                unique_predecessors.values()
            )

            if (
                len(predecessors)
                < 2
            ):

                continue

            tested_candidates += 1

            for index_a in range(
                len(predecessors)
            ):

                predecessor_a = (
                    predecessors[
                        index_a
                    ]
                )

                for index_b in range(
                    index_a + 1,
                    len(predecessors)
                ):

                    predecessor_b = (
                        predecessors[
                            index_b
                        ]
                    )

                    same_lane = (

                        predecessor_a.road_id
                        == predecessor_b.road_id

                        and

                        predecessor_a.lane_id
                        == predecessor_b.lane_id
                    )

                    if same_lane:
                        continue

                    predecessor_separation = (
                        predecessor_a
                        .transform
                        .location
                        .distance(
                            predecessor_b
                            .transform
                            .location
                        )
                    )

                    if (
                        predecessor_separation
                        < 5.0
                    ):

                        continue

                    path_a_to_merge = (
                        find_forward_path(

                            predecessor_a,

                            common_waypoint
                        )
                    )

                    path_b_to_merge = (
                        find_forward_path(

                            predecessor_b,

                            common_waypoint
                        )
                    )

                    if (
                        path_a_to_merge
                        is None

                        or

                        path_b_to_merge
                        is None
                    ):

                        continue

                    branch_a = (
                        extend_backward(

                            predecessor_a,

                            BRANCH_DISTANCE_M
                        )
                    )

                    branch_b = (
                        extend_backward(

                            predecessor_b,

                            BRANCH_DISTANCE_M
                        )
                    )

                    branch_a_length = (
                        get_route_length(
                            branch_a
                        )
                    )

                    branch_b_length = (
                        get_route_length(
                            branch_b
                        )
                    )

                    if (
                        branch_a_length
                        < 30.0

                        or

                        branch_b_length
                        < 30.0
                    ):

                        continue

                    start_separation = (
                        branch_a[0]
                        .transform
                        .location
                        .distance(
                            branch_b[0]
                            .transform
                            .location
                        )
                    )

                    if (
                        start_separation
                        < 12.0
                    ):

                        continue

                    route_a = (

                        branch_a

                        + path_a_to_merge[
                            1:
                        ]

                        + common_forward[
                            1:
                        ]
                    )

                    route_b = (

                        branch_b

                        + path_b_to_merge[
                            1:
                        ]

                        + common_forward[
                            1:
                        ]
                    )

                    merge_index_a = (

                        len(branch_a)

                        + len(
                            path_a_to_merge
                        )

                        - 2
                    )

                    merge_index_b = (

                        len(branch_b)

                        + len(
                            path_b_to_merge
                        )

                        - 2
                    )

                    route_a_length = (
                        get_route_length(
                            route_a
                        )
                    )

                    route_b_length = (
                        get_route_length(
                            route_b
                        )
                    )

                    if (
                        route_a_length
                        < 130.0

                        or

                        route_b_length
                        < 130.0
                    ):

                        continue

                    print(
                        f"Merge candidate löytyi "
                        f"(probe {probe_distance:.1f} m)."
                    )

                    print(
                        f"Testatut merge candidate -pisteet: "
                        f"{tested_candidates}"
                    )

                    return {

                        "route_a":
                            route_a,

                        "route_b":
                            route_b,

                        "common_route":
                            common_forward,

                        "merge_waypoint":
                            common_waypoint,

                        "merge_index_a":
                            merge_index_a,

                        "merge_index_b":
                            merge_index_b,

                        "start_separation":
                            start_separation,

                        "probe_distance":
                            probe_distance
                    }

    print(
        f"Testatut merge candidate -pisteet: "
        f"{tested_candidates}"
    )

    return None


# =========================================================
# DEBUG DRAWING
# =========================================================

def draw_route(
    world,
    route,
    color,
    thickness
):

    for index in range(
        len(route) - 1
    ):

        start = (
            route[index]
            .transform
            .location
        )

        end = (
            route[index + 1]
            .transform
            .location
        )

        world.debug.draw_line(

            carla.Location(
                x=start.x,
                y=start.y,
                z=start.z + 0.5
            ),

            carla.Location(
                x=end.x,
                y=end.y,
                z=end.z + 0.5
            ),

            thickness=thickness,

            color=color,

            life_time=100.0
        )


def draw_scenario(
    world,
    scenario,
    goal_index_a,
    goal_index_b
):

    route_a = (
        scenario[
            "route_a"
        ]
    )

    route_b = (
        scenario[
            "route_b"
        ]
    )

    common_route = (
        scenario[
            "common_route"
        ]
    )

    draw_route(
        world,
        route_a,
        carla.Color(
            0,
            255,
            255
        ),
        0.06
    )

    draw_route(
        world,
        route_b,
        carla.Color(
            255,
            150,
            0
        ),
        0.06
    )

    draw_route(
        world,
        common_route,
        carla.Color(
            255,
            0,
            255
        ),
        0.14
    )

    merge_location = (
        scenario[
            "merge_waypoint"
        ]
        .transform
        .location
    )

    world.debug.draw_point(

        carla.Location(
            x=merge_location.x,
            y=merge_location.y,
            z=merge_location.z + 1.0
        ),

        size=0.5,

        color=carla.Color(
            255,
            255,
            0
        ),

        life_time=100.0
    )

    goal_a = (
        route_a[
            goal_index_a
        ]
        .transform
        .location
    )

    goal_b = (
        route_b[
            goal_index_b
        ]
        .transform
        .location
    )

    world.debug.draw_point(

        carla.Location(
            x=goal_a.x,
            y=goal_a.y,
            z=goal_a.z + 1.0
        ),

        size=0.4,

        color=carla.Color(
            0,
            255,
            255
        ),

        life_time=100.0
    )

    world.debug.draw_point(

        carla.Location(
            x=goal_b.x,
            y=goal_b.y,
            z=goal_b.z + 1.0
        ),

        size=0.4,

        color=carla.Color(
            0,
            255,
            0
        ),

        life_time=100.0
    )


# =========================================================
# CONTINUOUS ROUTE PROJECTION
# =========================================================

def project_vehicle_to_route(
    vehicle,
    route,
    cumulative,
    current_index
):

    location = (
        vehicle.get_location()
    )

    search_start = max(
        0,
        current_index - 4
    )

    search_end = min(
        len(route) - 1,
        current_index + 20
    )

    best_distance_squared = float(
        "inf"
    )

    best_progress = (
        cumulative[
            current_index
        ]
    )

    best_route_index = (
        current_index
    )

    for segment_index in range(
        search_start,
        search_end
    ):

        point_a = (
            route[
                segment_index
            ]
            .transform
            .location
        )

        point_b = (
            route[
                segment_index + 1
            ]
            .transform
            .location
        )

        vx = (
            point_b.x
            - point_a.x
        )

        vy = (
            point_b.y
            - point_a.y
        )

        vz = (
            point_b.z
            - point_a.z
        )

        wx = (
            location.x
            - point_a.x
        )

        wy = (
            location.y
            - point_a.y
        )

        wz = (
            location.z
            - point_a.z
        )

        segment_length_squared = (
            vx * vx
            + vy * vy
            + vz * vz
        )

        if (
            segment_length_squared
            <= 0.000001
        ):

            continue

        t = (
            wx * vx
            + wy * vy
            + wz * vz
        ) / segment_length_squared

        t = clamp(
            t,
            0.0,
            1.0
        )

        projected_x = (
            point_a.x
            + t * vx
        )

        projected_y = (
            point_a.y
            + t * vy
        )

        projected_z = (
            point_a.z
            + t * vz
        )

        dx = (
            location.x
            - projected_x
        )

        dy = (
            location.y
            - projected_y
        )

        dz = (
            location.z
            - projected_z
        )

        distance_squared = (
            dx * dx
            + dy * dy
            + dz * dz
        )

        if (
            distance_squared
            < best_distance_squared
        ):

            best_distance_squared = (
                distance_squared
            )

            segment_length = (

                cumulative[
                    segment_index + 1
                ]

                - cumulative[
                    segment_index
                ]
            )

            best_progress = (

                cumulative[
                    segment_index
                ]

                + t
                * segment_length
            )

            if (
                t >= 0.5
            ):

                best_route_index = (
                    segment_index + 1
                )

            else:

                best_route_index = (
                    segment_index
                )

    best_route_index = max(
        current_index,
        best_route_index
    )

    return (
        best_route_index,
        best_progress,
        math.sqrt(
            best_distance_squared
        )
    )


# =========================================================
# STEERING
# =========================================================

def calculate_steering(
    vehicle,
    route,
    route_index,
    goal_index
):

    target_index = min(

        route_index
        + LOOKAHEAD_POINTS,

        goal_index
    )

    target_location = (
        route[
            target_index
        ]
        .transform
        .location
    )

    transform = (
        vehicle.get_transform()
    )

    location = (
        transform.location
    )

    dx = (
        target_location.x
        - location.x
    )

    dy = (
        target_location.y
        - location.y
    )

    target_yaw = math.degrees(
        math.atan2(
            dy,
            dx
        )
    )

    heading_error = (
        normalize_angle(

            target_yaw

            - transform.rotation.yaw
        )
    )

    return clamp(

        heading_error
        / 35.0,

        -1.0,

        1.0
    )


# =========================================================
# VEHICLE CONTROL
# =========================================================

def get_goal_distance(
    vehicle,
    route,
    goal_index
):

    return (
        vehicle
        .get_location()
        .distance(

            route[
                goal_index
            ]
            .transform
            .location
        )
    )


def get_target_speed_near_goal(
    cruise_speed_kmh,
    goal_distance_m
):

    if (
        goal_distance_m
        > 20.0
    ):

        return (
            cruise_speed_kmh
        )

    if (
        goal_distance_m
        > GOAL_TOLERANCE_M
    ):

        factor = clamp(

            goal_distance_m
            / 20.0,

            0.25,

            1.0
        )

        return (
            cruise_speed_kmh
            * factor
        )

    return 0.0


# =========================================================
# FOLLOW / COORDINATION SPEED
# =========================================================

def get_coordinated_follow_speed(
    cruise_speed_kmh,
    vehicle_b_speed_kmh,
    gap_m
):

    if (
        gap_m
        is None
    ):

        return (
            cruise_speed_kmh,
            False,
            False
        )


    # -----------------------------------------------------
    # B HAS STOPPED
    # -----------------------------------------------------
    #
    # Tämä korjaa edellisen testin ongelman.
    #
    # Jos B on jo loading zonessa pysähtyneenä,
    # A ei saa automaattisesti target speed = 0,
    # koska A:lla on oma tavoitepiste ennen B:tä.
    #
    # A saa ryömiä hitaasti eteenpäin, jos väli
    # on edelleen turvallinen.
    # -----------------------------------------------------

    if (
        vehicle_b_speed_kmh
        <= STOP_SPEED_KMH
    ):

        if (
            gap_m
            > STOPPED_LEAD_MIN_GAP_M
        ):

            return (
                LOADING_ZONE_CRAWL_SPEED_KMH,
                True,
                True
            )

        return (
            0.0,
            True,
            False
        )


    # -----------------------------------------------------
    # NORMAL FOLLOW COORDINATION
    # -----------------------------------------------------

    if (
        gap_m
        >= FOLLOW_FREE_GAP_M
    ):

        return (
            cruise_speed_kmh,
            False,
            False
        )


    if (
        gap_m
        <= FOLLOW_CLOSE_GAP_M
    ):

        target_speed = max(
            0.0,
            vehicle_b_speed_kmh
            - 4.0
        )

        return (
            target_speed,
            True,
            False
        )


    if (
        gap_m
        <= FOLLOW_MATCH_GAP_M
    ):

        return (
            max(
                0.0,
                vehicle_b_speed_kmh
            ),
            True,
            False
        )


    blend = (

        gap_m
        - FOLLOW_MATCH_GAP_M

    ) / (

        FOLLOW_FREE_GAP_M
        - FOLLOW_MATCH_GAP_M
    )

    blend = clamp(
        blend,
        0.0,
        1.0
    )

    following_speed = (
        vehicle_b_speed_kmh
        + 2.0
    )

    target_speed = (

        following_speed

        + blend
        * (
            cruise_speed_kmh
            - following_speed
        )
    )

    target_speed = clamp(
        target_speed,
        0.0,
        cruise_speed_kmh
    )

    return (
        target_speed,
        True,
        False
    )


def calculate_drive_control(
    speed_kmh,
    target_speed_kmh,
    steer
):

    error = (
        target_speed_kmh
        - speed_kmh
    )

    throttle = 0.0
    brake = 0.0

    if (
        error
        > 0.0
    ):

        throttle = clamp(

            error
            / 15.0,

            0.0,

            0.55
        )

    else:

        brake = clamp(

            (-error)
            / 10.0,

            0.0,

            0.7
        )

    return carla.VehicleControl(
        throttle=throttle,
        brake=brake,
        steer=steer
    )


# =========================================================
# CONTINUOUS COMMON-ROUTE GAP
# =========================================================

def get_common_route_gap(
    vehicle_a,
    vehicle_b,
    progress_a,
    progress_b,
    merge_distance_a,
    merge_distance_b
):

    if (
        progress_a
        < merge_distance_a - 1.0

        or

        progress_b
        < merge_distance_b - 1.0
    ):

        return None

    common_progress_a = (
        progress_a
        - merge_distance_a
    )

    common_progress_b = (
        progress_b
        - merge_distance_b
    )

    if (
        common_progress_b
        <= common_progress_a
    ):

        return None

    center_distance = (
        common_progress_b
        - common_progress_a
    )

    half_length_a = (
        vehicle_a
        .bounding_box
        .extent
        .x
    )

    half_length_b = (
        vehicle_b
        .bounding_box
        .extent
        .x
    )

    bumper_gap = (

        center_distance

        - half_length_a

        - half_length_b
    )

    return max(
        0.0,
        bumper_gap
    )


# =========================================================
# ZEROMQ
# =========================================================

def drain_socket(
    socket
):

    while True:

        try:

            socket.recv_string(
                flags=zmq.NOBLOCK
            )

        except zmq.Again:

            break


def get_controller_command(
    sensor_socket,
    control_socket,
    poller,
    sensor_data
):

    expected_test = (
        sensor_data[
            "test_id"
        ]
    )

    expected_sequence = (
        sensor_data[
            "sequence"
        ]
    )

    start_time = (
        time.perf_counter()
    )

    for _ in range(3):

        sensor_socket.send_string(
            json.dumps(
                sensor_data
            )
        )

        events = dict(
            poller.poll(
                200
            )
        )

        if (
            control_socket
            not in events
        ):

            continue

        while True:

            try:

                message = (
                    control_socket
                    .recv_string(
                        flags=zmq.NOBLOCK
                    )
                )

                command = (
                    json.loads(
                        message
                    )
                )

                if (
                    command.get(
                        "test_id"
                    )
                    == expected_test

                    and

                    command.get(
                        "sequence"
                    )
                    == expected_sequence
                ):

                    round_trip_ms = (

                        time.perf_counter()
                        - start_time

                    ) * 1000.0

                    return (
                        command,
                        round_trip_ms
                    )

            except zmq.Again:

                break

    return None, None


# =========================================================
# CSV
# =========================================================

def save_result(
    result
):

    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    file_exists = (
        RESULTS_FILE.exists()
    )

    fieldnames = [

        "timestamp",

        "route_a_length_m",

        "route_b_length_m",

        "start_separation_m",

        "merge_probe_distance_m",

        "vehicle_a_speed_kmh",

        "vehicle_b_speed_kmh",

        "vehicle_a_start_delay_s",

        "safety_interrupt_count",

        "first_trigger_speed_a_kmh",

        "first_trigger_speed_b_kmh",

        "first_trigger_relative_speed_kmh",

        "first_trigger_gap_m",

        "first_trigger_ttc_s",

        "minimum_gap_m",

        "vehicle_a_merged",

        "vehicle_b_merged",

        "vehicle_a_goal_reached",

        "vehicle_b_goal_reached",

        "collision_a",

        "collision_b",

        "controller_processing_ms",

        "controller_round_trip_ms",

        "result"
    ]

    with RESULTS_FILE.open(
        "a",
        newline="",
        encoding="utf-8"
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames
        )

        if not file_exists:

            writer.writeheader()

        writer.writerow(
            result
        )


# =========================================================
# MAIN
# =========================================================

def main():

    world = None
    original_settings = None

    vehicle_a = None
    vehicle_b = None

    collision_sensor_a = None
    collision_sensor_b = None

    context = None
    sensor_socket = None
    control_socket = None


    collision_state = {
        "A": False,
        "B": False
    }


    sequence = 0
    simulation_time = 0.0


    route_index_a = 0
    route_index_b = 0

    progress_a = 0.0
    progress_b = 0.0


    vehicle_a_started = False

    vehicle_a_merged = False
    vehicle_b_merged = False

    vehicle_a_goal_reached = False
    vehicle_b_goal_reached = False


    safety_latched = False
    safety_interrupt_count = 0


    merge_gate_active = False
    previous_merge_gate_active = False


    follow_active = False
    previous_follow_active = False


    loading_crawl_active = False
    previous_loading_crawl_active = False


    minimum_gap = None


    first_trigger_speed_a = None
    first_trigger_speed_b = None
    first_trigger_relative_speed = None
    first_trigger_gap = None
    first_trigger_ttc = None

    first_controller_processing_ms = None
    first_controller_round_trip_ms = None


    try:

        # =================================================
        # ZEROMQ
        # =================================================

        context = (
            zmq.Context()
        )

        sensor_socket = (
            context.socket(
                zmq.PUB
            )
        )

        sensor_socket.bind(
            SENSOR_ADDRESS
        )


        control_socket = (
            context.socket(
                zmq.SUB
            )
        )

        control_socket.connect(
            CONTROL_ADDRESS
        )

        control_socket.setsockopt_string(
            zmq.SUBSCRIBE,
            ""
        )


        poller = (
            zmq.Poller()
        )

        poller.register(
            control_socket,
            zmq.POLLIN
        )


        # =================================================
        # CARLA
        # =================================================

        print(
            "Yhdistetään CARLAan..."
        )

        client = carla.Client(
            "localhost",
            2000
        )

        client.set_timeout(
            10.0
        )

        world = (
            client.get_world()
        )

        print(
            "Yhteys CARLAan onnistui."
        )

        print(
            f"Kartta: "
            f"{world.get_map().name}"
        )


        # =================================================
        # SYNCHRONOUS MODE
        # =================================================

        original_settings = (
            world.get_settings()
        )

        settings = (
            world.get_settings()
        )

        settings.synchronous_mode = True

        settings.fixed_delta_seconds = (
            FIXED_DELTA_SECONDS
        )

        world.apply_settings(
            settings
        )

        world.set_weather(
            carla.WeatherParameters.ClearNoon
        )

        print(
            f"Synchronous mode: "
            f"{1 / FIXED_DELTA_SECONDS:.1f} Hz"
        )


        # =================================================
        # FIND SCENARIO
        # =================================================

        print()

        scenario = (
            find_merge_scenario(
                world
            )
        )

        if (
            scenario
            is None
        ):

            raise RuntimeError(

                "Sopivaa merge-kohtaa ei löytynyt "
                "Town10HD-kartasta."
            )


        route_a = (
            scenario[
                "route_a"
            ]
        )

        route_b = (
            scenario[
                "route_b"
            ]
        )


        merge_index_a = (
            scenario[
                "merge_index_a"
            ]
        )

        merge_index_b = (
            scenario[
                "merge_index_b"
            ]
        )


        cumulative_a = (
            build_cumulative_distances(
                route_a
            )
        )

        cumulative_b = (
            build_cumulative_distances(
                route_b
            )
        )


        route_length_a = (
            cumulative_a[-1]
        )

        route_length_b = (
            cumulative_b[-1]
        )


        merge_distance_a = (
            cumulative_a[
                merge_index_a
            ]
        )

        merge_distance_b = (
            cumulative_b[
                merge_index_b
            ]
        )


        # =================================================
        # GOALS
        # =================================================

        goal_index_b = (
            len(route_b)
            - 1
        )


        goal_index_a = (
            find_index_before_end(

                route_a,

                VEHICLE_A_GOAL_OFFSET_M
            )
        )


        goal_progress_a = (
            cumulative_a[
                goal_index_a
            ]
        )

        goal_progress_b = (
            cumulative_b[
                goal_index_b
            ]
        )


        print()
        print(
            "Merge-skenaario löytyi."
        )

        print(
            f"Route A length: "
            f"{route_length_a:.1f} m"
        )

        print(
            f"Route B length: "
            f"{route_length_b:.1f} m"
        )

        print(
            f"Start separation: "
            f"{scenario['start_separation']:.1f} m"
        )

        print(
            f"Merge probe distance: "
            f"{scenario['probe_distance']:.1f} m"
        )

        print(
            f"Merge distance A: "
            f"{merge_distance_a:.1f} m"
        )

        print(
            f"Merge distance B: "
            f"{merge_distance_b:.1f} m"
        )


        # =================================================
        # START-TIME COORDINATION
        # =================================================

        estimated_time_a = (

            merge_distance_a

            / (
                VEHICLE_A_SPEED_KMH
                / 3.6
            )
        )


        estimated_time_b = (

            merge_distance_b

            / (
                VEHICLE_B_SPEED_KMH
                / 3.6
            )
        )


        vehicle_a_start_delay = max(

            0.0,

            estimated_time_b

            + MERGE_HEADWAY_SECONDS

            - estimated_time_a
        )


        print()
        print(
            f"Estimated A time to merge: "
            f"{estimated_time_a:.2f} s"
        )

        print(
            f"Estimated B time to merge: "
            f"{estimated_time_b:.2f} s"
        )

        print(
            f"Vehicle A start delay: "
            f"{vehicle_a_start_delay:.2f} s"
        )


        # =================================================
        # DRAW
        # =================================================

        draw_scenario(

            world,

            scenario,

            goal_index_a,

            goal_index_b
        )


        # =================================================
        # SPAWN VEHICLES
        # =================================================

        blueprint_library = (
            world.get_blueprint_library()
        )


        vehicle_bp_a = (
            blueprint_library.find(
                "vehicle.tesla.model3"
            )
        )

        if (
            vehicle_bp_a.has_attribute(
                "color"
            )
        ):

            vehicle_bp_a.set_attribute(
                "color",
                "0,0,255"
            )


        vehicle_bp_b = (
            blueprint_library.find(
                "vehicle.tesla.model3"
            )
        )

        if (
            vehicle_bp_b.has_attribute(
                "color"
            )
        ):

            vehicle_bp_b.set_attribute(
                "color",
                "255,0,0"
            )


        vehicle_a = (
            world.try_spawn_actor(

                vehicle_bp_a,

                make_spawn_transform(
                    route_a[0]
                )
            )
        )

        if (
            vehicle_a
            is None
        ):

            raise RuntimeError(
                "Vehicle A:n spawn epäonnistui."
            )


        vehicle_b = (
            world.try_spawn_actor(

                vehicle_bp_b,

                make_spawn_transform(
                    route_b[0]
                )
            )
        )

        if (
            vehicle_b
            is None
        ):

            raise RuntimeError(
                "Vehicle B:n spawn epäonnistui."
            )


        # =================================================
        # COLLISION SENSORS
        # =================================================

        collision_bp = (
            blueprint_library.find(
                "sensor.other.collision"
            )
        )


        collision_sensor_a = (
            world.spawn_actor(

                collision_bp,

                carla.Transform(),

                attach_to=vehicle_a
            )
        )


        collision_sensor_b = (
            world.spawn_actor(

                collision_bp,

                carla.Transform(),

                attach_to=vehicle_b
            )
        )


        def collision_callback_a(
            event
        ):

            collision_state[
                "A"
            ] = True


        def collision_callback_b(
            event
        ):

            collision_state[
                "B"
            ] = True


        collision_sensor_a.listen(
            collision_callback_a
        )

        collision_sensor_b.listen(
            collision_callback_b
        )


        # =================================================
        # INITIALIZATION
        # =================================================

        for _ in range(5):

            world.tick()


        time.sleep(
            1.0
        )

        drain_socket(
            control_socket
        )


        print()
        print(
            "=" * 100
        )

        print(
            "TWO ROUTES -> MERGE COORDINATION -> COMMON LOADING ZONE"
        )

        print(
            "=" * 100
        )

        print(
            f"Vehicle A cruise speed: "
            f"{VEHICLE_A_SPEED_KMH:.1f} km/h"
        )

        print(
            f"Vehicle B cruise speed: "
            f"{VEHICLE_B_SPEED_KMH:.1f} km/h"
        )

        print()
        print(
            "Coordination layers:"
        )

        print(
            "1. Start-time coordination"
        )

        print(
            "2. Merge gate"
        )

        print(
            "3. Post-merge speed coordination"
        )

        print(
            "4. Loading-zone crawl"
        )

        print(
            "5. TTC/AEB safety override"
        )

        print()


        # =================================================
        # TEST LOOP
        # =================================================

        while (
            simulation_time
            < TEST_TIMEOUT_SECONDS
        ):

            world.tick()

            simulation_time += (
                FIXED_DELTA_SECONDS
            )

            time.sleep(
                0.001
            )


            # =================================================
            # SPEEDS
            # =================================================

            speed_a = (
                get_speed_kmh(
                    vehicle_a
                )
            )

            speed_b = (
                get_speed_kmh(
                    vehicle_b
                )
            )


            # =================================================
            # CONTINUOUS ROUTE PROGRESS
            # =================================================

            (
                route_index_a,
                progress_a,
                route_error_a
            ) = project_vehicle_to_route(

                vehicle_a,

                route_a,

                cumulative_a,

                route_index_a
            )


            (
                route_index_b,
                progress_b,
                route_error_b
            ) = project_vehicle_to_route(

                vehicle_b,

                route_b,

                cumulative_b,

                route_index_b
            )


            # =================================================
            # START VEHICLE A
            # =================================================

            if (
                not vehicle_a_started

                and

                simulation_time
                >= vehicle_a_start_delay
            ):

                vehicle_a_started = True

                print()
                print()

                print(
                    "*** VEHICLE A START ***"
                )

                print(
                    f"Simulation time: "
                    f"{simulation_time:.2f} s"
                )


            # =================================================
            # MERGE PROGRESS
            # =================================================

            b_common_lead = None

            if (
                progress_b
                >= merge_distance_b
            ):

                b_common_lead = (
                    progress_b
                    - merge_distance_b
                )


            distance_a_to_merge = (
                merge_distance_a
                - progress_a
            )


            if (
                not vehicle_a_merged

                and

                progress_a
                >= merge_distance_a - 0.5
            ):

                vehicle_a_merged = True

                print()
                print()

                print(
                    "*** VEHICLE A ENTERED COMMON ROUTE ***"
                )


            if (
                not vehicle_b_merged

                and

                progress_b
                >= merge_distance_b - 0.5
            ):

                vehicle_b_merged = True

                print()
                print()

                print(
                    "*** VEHICLE B ENTERED COMMON ROUTE ***"
                )


            # =================================================
            # MERGE GATE
            # =================================================

            merge_gate_active = False


            if (
                vehicle_a_started

                and not vehicle_a_merged

                and distance_a_to_merge
                <= MERGE_GATE_DISTANCE_M
            ):

                if (
                    b_common_lead
                    is None

                    or

                    b_common_lead
                    < MERGE_RELEASE_LEAD_M
                ):

                    merge_gate_active = True


            if (
                merge_gate_active

                and not previous_merge_gate_active
            ):

                print()
                print()

                print(
                    "*** MERGE GATE ACTIVE ***"
                )

                print(
                    "Vehicle A odottaa ennen mergeä."
                )


            if (
                not merge_gate_active

                and previous_merge_gate_active
            ):

                print()
                print()

                print(
                    "*** MERGE GATE RELEASED ***"
                )

                if (
                    b_common_lead
                    is not None
                ):

                    print(
                        f"Vehicle B common-route lead: "
                        f"{b_common_lead:.2f} m"
                    )


            previous_merge_gate_active = (
                merge_gate_active
            )


            # =================================================
            # GOAL DISTANCES
            # =================================================

            goal_distance_a = (
                get_goal_distance(

                    vehicle_a,

                    route_a,

                    goal_index_a
                )
            )


            goal_distance_b = (
                get_goal_distance(

                    vehicle_b,

                    route_b,

                    goal_index_b
                )
            )


            # =================================================
            # GOAL DETECTION
            # =================================================

            if (
                not vehicle_a_goal_reached

                and

                goal_distance_a
                <= GOAL_TOLERANCE_M

                and

                progress_a
                >= goal_progress_a - 4.0
            ):

                vehicle_a_goal_reached = True

                safety_latched = False

                print()
                print()

                print(
                    "*** VEHICLE A REACHED "
                    "LOADING ZONE ***"
                )

                print(
                    f"Goal distance A: "
                    f"{goal_distance_a:.2f} m"
                )


            if (
                not vehicle_b_goal_reached

                and

                goal_distance_b
                <= GOAL_TOLERANCE_M

                and

                progress_b
                >= goal_progress_b - 4.0
            ):

                vehicle_b_goal_reached = True

                print()
                print()

                print(
                    "*** VEHICLE B REACHED "
                    "LOADING ZONE ***"
                )

                print(
                    f"Goal distance B: "
                    f"{goal_distance_b:.2f} m"
                )


            # =================================================
            # CONTINUOUS COMMON-ROUTE GAP
            # =================================================

            gap = (
                get_common_route_gap(

                    vehicle_a,

                    vehicle_b,

                    progress_a,

                    progress_b,

                    merge_distance_a,

                    merge_distance_b
                )
            )


            if (
                gap
                is not None
            ):

                if (
                    minimum_gap
                    is None

                    or

                    gap
                    < minimum_gap
                ):

                    minimum_gap = (
                        gap
                    )


            # =================================================
            # TTC SAFETY CONTROLLER
            # =================================================

            sequence += 1


            sensor_data = {

                "test_id":
                    "two_route_merge",

                "sequence":
                    sequence,

                "ego_speed_kmh":
                    speed_a,

                "obstacle_speed_kmh":
                    speed_b,

                "obstacle_distance_m":
                    gap
            }


            (
                command,
                round_trip_ms
            ) = get_controller_command(

                sensor_socket,

                control_socket,

                poller,

                sensor_data
            )


            if (
                command
                is None
            ):

                raise RuntimeError(
                    "TTC-controllerilta ei saatu vastausta."
                )


            aeb_active = bool(
                command.get(
                    "aeb_active",
                    False
                )
            )


            current_ttc = (
                command.get(
                    "ttc_s"
                )
            )


            relative_speed = (
                command.get(
                    "relative_speed_kmh"
                )
            )


            # =================================================
            # SAFETY INTERRUPT
            # =================================================

            if (
                aeb_active

                and not safety_latched

                and vehicle_a_started

                and not vehicle_a_goal_reached
            ):

                safety_latched = True

                safety_interrupt_count += 1

                print()
                print()

                print(
                    "*** VEHICLE A SAFETY INTERRUPT ***"
                )

                print(
                    f"Event: "
                    f"{safety_interrupt_count}"
                )

                print(
                    f"A speed: "
                    f"{speed_a:.2f} km/h"
                )

                print(
                    f"B speed: "
                    f"{speed_b:.2f} km/h"
                )

                print(
                    f"Relative speed: "
                    f"{relative_speed:.2f} km/h"
                )

                print(
                    f"Continuous gap: "
                    f"{gap:.2f} m"
                )

                print(
                    f"TTC: "
                    f"{current_ttc:.3f} s"
                )


                if (
                    safety_interrupt_count
                    == 1
                ):

                    first_trigger_speed_a = (
                        speed_a
                    )

                    first_trigger_speed_b = (
                        speed_b
                    )

                    first_trigger_relative_speed = (
                        relative_speed
                    )

                    first_trigger_gap = (
                        gap
                    )

                    first_trigger_ttc = (
                        current_ttc
                    )

                    first_controller_processing_ms = (
                        command.get(
                            "controller_processing_ms"
                        )
                    )

                    first_controller_round_trip_ms = (
                        round_trip_ms
                    )


            # =================================================
            # SAFETY RELEASE
            # =================================================

            if (
                safety_latched

                and not aeb_active

                and speed_a
                <= STOP_SPEED_KMH

                and gap
                is not None

                and gap
                >= SAFETY_RELEASE_GAP_M

                and not vehicle_a_goal_reached
            ):

                safety_latched = False

                print()
                print()

                print(
                    "*** VEHICLE A SAFETY RELEASED ***"
                )

                print(
                    f"Safe gap: "
                    f"{gap:.2f} m"
                )


            # =================================================
            # STEERING
            # =================================================

            steer_a = (
                calculate_steering(

                    vehicle_a,

                    route_a,

                    route_index_a,

                    goal_index_a
                )
            )


            steer_b = (
                calculate_steering(

                    vehicle_b,

                    route_b,

                    route_index_b,

                    goal_index_b
                )
            )


            # =================================================
            # VEHICLE B CONTROL
            # =================================================

            if (
                vehicle_b_goal_reached
            ):

                vehicle_b.apply_control(

                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

            else:

                target_b = (
                    get_target_speed_near_goal(

                        VEHICLE_B_SPEED_KMH,

                        goal_distance_b
                    )
                )

                vehicle_b.apply_control(

                    calculate_drive_control(

                        speed_b,

                        target_b,

                        steer_b
                    )
                )


            # =================================================
            # VEHICLE A NORMAL GOAL SPEED
            # =================================================

            normal_target_a = (
                get_target_speed_near_goal(

                    VEHICLE_A_SPEED_KMH,

                    goal_distance_a
                )
            )


            # =================================================
            # POST-MERGE FOLLOW COORDINATION
            # =================================================

            (
                coordinated_target_a,
                follow_active,
                loading_crawl_active
            ) = get_coordinated_follow_speed(

                VEHICLE_A_SPEED_KMH,

                speed_b,

                gap
            )


            target_a = min(
                normal_target_a,
                coordinated_target_a
            )


            # =================================================
            # FOLLOW STATE OUTPUT
            # =================================================

            if (
                follow_active

                and not previous_follow_active
            ):

                print()
                print()

                print(
                    "*** SPEED COORDINATION ACTIVE ***"
                )

                if (
                    gap
                    is not None
                ):

                    print(
                        f"Gap: "
                        f"{gap:.2f} m"
                    )


            if (
                not follow_active

                and previous_follow_active
            ):

                print()
                print()

                print(
                    "*** SPEED COORDINATION RELEASED ***"
                )


            previous_follow_active = (
                follow_active
            )


            # =================================================
            # LOADING-ZONE CRAWL OUTPUT
            # =================================================

            if (
                loading_crawl_active

                and not previous_loading_crawl_active
            ):

                print()
                print()

                print(
                    "*** LOADING ZONE CRAWL ACTIVE ***"
                )

                print(
                    "Vehicle B on pysähtynyt."
                )

                print(
                    "Vehicle A jatkaa hitaasti "
                    "omaan tavoitepisteeseensä."
                )


            if (
                not loading_crawl_active

                and previous_loading_crawl_active
            ):

                print()
                print()

                print(
                    "*** LOADING ZONE CRAWL RELEASED ***"
                )


            previous_loading_crawl_active = (
                loading_crawl_active
            )


            # =================================================
            # VEHICLE A CONTROL
            # =================================================

            if (
                not vehicle_a_started
            ):

                vehicle_a.apply_control(

                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )


            elif (
                vehicle_a_goal_reached
            ):

                vehicle_a.apply_control(

                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )


            elif (
                safety_latched
            ):

                vehicle_a.apply_control(

                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=steer_a
                    )
                )


            elif (
                merge_gate_active
            ):

                vehicle_a.apply_control(

                    calculate_drive_control(

                        speed_a,

                        0.0,

                        steer_a
                    )
                )


            else:

                vehicle_a.apply_control(

                    calculate_drive_control(

                        speed_a,

                        target_a,

                        steer_a
                    )
                )


            # =================================================
            # COLLISION
            # =================================================

            if (
                collision_state[
                    "A"
                ]

                or

                collision_state[
                    "B"
                ]
            ):

                vehicle_a.apply_control(

                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

                vehicle_b.apply_control(

                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0
                    )
                )

                print()
                print()

                print(
                    "*** COLLISION DETECTED ***"
                )

                break


            # =================================================
            # SUCCESS EXIT
            # =================================================

            if (
                vehicle_a_goal_reached

                and

                vehicle_b_goal_reached
            ):

                for _ in range(3):

                    world.tick()

                print()
                print()

                print(
                    "*** BOTH VEHICLES REACHED "
                    "COMMON LOADING ZONE ***"
                )

                break


            # =================================================
            # TERMINAL STATE
            # =================================================

            if (
                not vehicle_a_started
            ):

                state_a = (
                    "WAIT"
                )

            elif (
                vehicle_a_goal_reached
            ):

                state_a = (
                    "GOAL"
                )

            elif (
                safety_latched
            ):

                state_a = (
                    "HOLD"
                )

            elif (
                merge_gate_active
            ):

                state_a = (
                    "GATE"
                )

            elif (
                loading_crawl_active
            ):

                state_a = (
                    "CRAWL"
                )

            elif (
                follow_active
            ):

                state_a = (
                    "FOLLOW"
                )

            else:

                state_a = (
                    "NAV"
                )


            if (
                vehicle_b_goal_reached
            ):

                state_b = (
                    "GOAL"
                )

            else:

                state_b = (
                    "NAV"
                )


            gap_text = (
                "---"

                if gap
                is None

                else
                f"{gap:.1f}"
            )


            ttc_text = (
                "---"

                if current_ttc
                is None

                else
                f"{current_ttc:.2f}"
            )


            target_text = (
                "---"

                if not vehicle_a_started

                else
                f"{target_a:.1f}"
            )


            print(

                f"A:{state_a:6} "
                f"{speed_a:5.1f} km/h "
                f"target {target_text:>5} | "

                f"B:{state_b:4} "
                f"{speed_b:5.1f} km/h | "

                f"Gap: "
                f"{gap_text:>5} m | "

                f"TTC: "
                f"{ttc_text:>5} s | "

                f"Goal A: "
                f"{goal_distance_a:5.1f} m | "

                f"RouteErr A/B: "
                f"{route_error_a:4.1f}/"
                f"{route_error_b:4.1f} m   ",

                end="\r"
            )


        # =================================================
        # RESULT
        # =================================================

        final_goal_distance_a = (
            get_goal_distance(

                vehicle_a,

                route_a,

                goal_index_a
            )
        )


        final_goal_distance_b = (
            get_goal_distance(

                vehicle_b,

                route_b,

                goal_index_b
            )
        )


        collision_free = (

            not collision_state[
                "A"
            ]

            and

            not collision_state[
                "B"
            ]
        )


        safety_ok = (

            minimum_gap
            is not None

            and

            minimum_gap
            >= SAFETY_MARGIN_M
        )


        success = (

            vehicle_a_merged

            and

            vehicle_b_merged

            and

            vehicle_a_goal_reached

            and

            vehicle_b_goal_reached

            and

            collision_free

            and

            safety_ok
        )


        result_string = (
            "PASS"

            if success

            else "FAIL"
        )


        print()
        print()

        print(
            "=" * 100
        )

        print(
            "TEST RESULT"
        )

        print(
            "=" * 100
        )


        print(
            f"Vehicle A merged: "
            f"{vehicle_a_merged}"
        )

        print(
            f"Vehicle B merged: "
            f"{vehicle_b_merged}"
        )

        print(
            f"Safety interrupts: "
            f"{safety_interrupt_count}"
        )

        print(
            f"Minimum continuous gap: "
            f"{minimum_gap}"
        )

        print(
            f"Vehicle A goal reached: "
            f"{vehicle_a_goal_reached}"
        )

        print(
            f"Vehicle B goal reached: "
            f"{vehicle_b_goal_reached}"
        )

        print(
            f"Vehicle A final goal distance: "
            f"{final_goal_distance_a:.2f} m"
        )

        print(
            f"Vehicle B final goal distance: "
            f"{final_goal_distance_b:.2f} m"
        )

        print(
            f"Vehicle A collision: "
            f"{collision_state['A']}"
        )

        print(
            f"Vehicle B collision: "
            f"{collision_state['B']}"
        )


        if (
            safety_interrupt_count
            == 0
        ):

            print()
            print(
                "Coordination prevented the need "
                "for emergency braking."
            )


        print()
        print(
            f"RESULT: "
            f"{result_string}"
        )


        # =================================================
        # CSV
        # =================================================

        result = {

            "timestamp":
                datetime.now().isoformat(
                    timespec="seconds"
                ),

            "route_a_length_m":
                round(
                    route_length_a,
                    3
                ),

            "route_b_length_m":
                round(
                    route_length_b,
                    3
                ),

            "start_separation_m":
                round(
                    scenario[
                        "start_separation"
                    ],
                    3
                ),

            "merge_probe_distance_m":
                scenario[
                    "probe_distance"
                ],

            "vehicle_a_speed_kmh":
                VEHICLE_A_SPEED_KMH,

            "vehicle_b_speed_kmh":
                VEHICLE_B_SPEED_KMH,

            "vehicle_a_start_delay_s":
                round(
                    vehicle_a_start_delay,
                    3
                ),

            "safety_interrupt_count":
                safety_interrupt_count,

            "first_trigger_speed_a_kmh":
                (
                    round(
                        first_trigger_speed_a,
                        3
                    )
                    if first_trigger_speed_a
                    is not None
                    else ""
                ),

            "first_trigger_speed_b_kmh":
                (
                    round(
                        first_trigger_speed_b,
                        3
                    )
                    if first_trigger_speed_b
                    is not None
                    else ""
                ),

            "first_trigger_relative_speed_kmh":
                (
                    round(
                        first_trigger_relative_speed,
                        3
                    )
                    if first_trigger_relative_speed
                    is not None
                    else ""
                ),

            "first_trigger_gap_m":
                (
                    round(
                        first_trigger_gap,
                        3
                    )
                    if first_trigger_gap
                    is not None
                    else ""
                ),

            "first_trigger_ttc_s":
                (
                    round(
                        first_trigger_ttc,
                        4
                    )
                    if first_trigger_ttc
                    is not None
                    else ""
                ),

            "minimum_gap_m":
                (
                    round(
                        minimum_gap,
                        3
                    )
                    if minimum_gap
                    is not None
                    else ""
                ),

            "vehicle_a_merged":
                vehicle_a_merged,

            "vehicle_b_merged":
                vehicle_b_merged,

            "vehicle_a_goal_reached":
                vehicle_a_goal_reached,

            "vehicle_b_goal_reached":
                vehicle_b_goal_reached,

            "collision_a":
                collision_state[
                    "A"
                ],

            "collision_b":
                collision_state[
                    "B"
                ],

            "controller_processing_ms":
                (
                    round(
                        first_controller_processing_ms,
                        3
                    )
                    if first_controller_processing_ms
                    is not None
                    else ""
                ),

            "controller_round_trip_ms":
                (
                    round(
                        first_controller_round_trip_ms,
                        3
                    )
                    if first_controller_round_trip_ms
                    is not None
                    else ""
                ),

            "result":
                result_string
        }


        save_result(
            result
        )


        print()
        print(
            "Tulokset:"
        )

        print(
            RESULTS_FILE
        )


    except KeyboardInterrupt:

        print()
        print(
            "Testi keskeytetty."
        )


    except Exception as error:

        print()
        print(
            f"Virhe: "
            f"{error}"
        )


    finally:

        # =================================================
        # CLEANUP
        # =================================================

        if (
            collision_sensor_a
            is not None
        ):

            collision_sensor_a.stop()
            collision_sensor_a.destroy()


        if (
            collision_sensor_b
            is not None
        ):

            collision_sensor_b.stop()
            collision_sensor_b.destroy()


        if (
            vehicle_a
            is not None
        ):

            vehicle_a.destroy()


        if (
            vehicle_b
            is not None
        ):

            vehicle_b.destroy()


        if (
            world
            is not None

            and

            original_settings
            is not None
        ):

            print()
            print(
                "Palautetaan CARLAn asetukset..."
            )

            world.apply_settings(
                original_settings
            )


        if (
            sensor_socket
            is not None
        ):

            sensor_socket.close()


        if (
            control_socket
            is not None
        ):

            control_socket.close()


        if (
            context
            is not None
        ):

            context.term()


        print(
            "Valmis."
        )


if __name__ == "__main__":

    main()