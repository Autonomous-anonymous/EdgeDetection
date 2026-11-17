"""Presentation markings and the Gazebo view for the integrated showcase."""
import math

from jarcrac_cooperative_control import Curve
from jarcrac_cooperative_sil import add
from jarcrac_showcase_mission import MERGE_APPROACH_END

MERGE_COLOR = (.98, .73, .18, 1.)
ROUTE_COLOR = (.88, .88, .79, 1.)
PARK_COLOR = ((.26, .86, .52, 1.), (.26, .65, 1., 1.))


def paint(world, terrain, frame, name, color, strokes):
    """Follow the ground with short, visual-only paint strips."""
    model = add(world, "model", name=name)
    add(model, "static", "true")
    link = add(model, "link", name="paint")
    for segment, (start, end, width) in enumerate(strokes):
        dx, dy = end[0]-start[0], end[1]-start[1]
        length = math.hypot(dx, dy)
        count = max(1, math.ceil(length/.42))
        yaw = frame.yaw+math.atan2(dy, dx)
        for i in range(count):
            u = (i+.5)/count
            x, y = frame.world(start[0]+u*dx, start[1]+u*dy)
            ground = terrain.height_at(x, y)
            if ground is None:
                raise ValueError(f"{name} falls outside the terrain mesh")
            item = add(link, "visual", name=f"stripe_{segment}_{i}")
            add(item, "pose", f"{x} {y} {ground+.032} 0 0 {yaw}")
            box = add(add(item, "geometry"), "box")
            add(box, "size", f"{length/count+.004} {width} .025")
            material = add(item, "material")
            rgb = " ".join(map(str, color))
            add(material, "ambient", rgb)
            add(material, "diffuse", rgb)
            add(item, "cast_shadows", "false")


def outline(vertices, width):
    return [(start, end, width) for start, end in zip(vertices, (*vertices[1:], vertices[0]))]


def showcase_scene(world, terrain, trial, *, show_lidar):
    frame = trial.frame
    world.find("scene/shadows").text = "false"
    paint(world, terrain, frame, "merge_zone_marker", MERGE_COLOR, outline(trial.merge_zone, .22))
    for label, goal, color in zip("ab", trial.goals, PARK_COLOR):
        bay = ((goal-1.75, -1.22), (goal+1.75, -1.22), (goal+1.75, 1.22), (goal-1.75, 1.22))
        gate = ((goal-.95, -.83), (goal-.95, .83))
        paint(world, terrain, frame, f"loading_zone_{label}", color, [*outline(bay, .22), (*gate, .13)])

    road = [((x, 0.), (x+.65, 0.), .10) for x in range(math.ceil(trial.starts[0]), int(trial.goals[1])-2, 2)]
    approach = Curve(*trial.start(1), math.tan(trial.start_yaw[1]), MERGE_APPROACH_END, 0.)
    road.extend((approach.at(x)[:2], approach.at(x+.65)[:2], .10)
                for x in range(math.ceil(trial.starts[1]), math.floor(MERGE_APPROACH_END), 2))
    paint(world, terrain, frame, "route_markers", ROUTE_COLOR, road)

    previous = world.find("gui")
    if previous is not None:
        world.remove(previous)
    gui = add(world, "gui", fullscreen="false")
    view = add(gui, "plugin", filename="MinimalScene", name="3D View")
    props = add(view, "gz-gui")
    add(props, "property", "false", type="bool").set("key", "showTitleBar")
    add(props, "property", "docked", type="string").set("key", "state")
    add(view, "engine", "ogre2")
    add(view, "scene", "scene")
    center = (min(trial.starts)+max(trial.goals))/2
    x, y = frame.world(center, -30.)
    z = terrain.height_at(*frame.world(center, 0.))+36.
    add(view, "camera_pose", f"{x} {y} {z} 0 {math.atan2(36., 30.)} {frame.yaw+math.pi/2}")
    for name, title in (("GzSceneManager", "Scene Manager"),
                        ("InteractiveViewControl", "Interactive View Control")):
        plugin = add(gui, "plugin", filename=name, name=title)
        props = add(plugin, "gz-gui")
        for key, value, type_name in (("resizable", "false", "bool"), ("width", "5", "double"),
                                      ("height", "5", "double"), ("state", "floating", "string"),
                                      ("showTitleBar", "false", "bool")):
            add(props, "property", value, type=type_name).set("key", key)
    # Restore the RTF, simulation time, and real time visible in the standard
    # Gazebo UI.  The showcase's custom <gui> replaces the default plugins.
    stats = add(gui, "plugin", filename="WorldStats", name="World stats")
    settings = add(stats, "gz-gui")
    for key, value, kind in (("showTitleBar", "false", "bool"),
                             ("state", "floating", "string"),
                             ("width", "290", "double"),
                             ("height", "110", "double"),
                             ("z", "1", "double")):
        add(settings, "property", value, type=kind).set("key", key)
    anchors = add(settings, "anchors", target="3D View")
    add(anchors, "line", own="right", target="right")
    add(anchors, "line", own="bottom", target="bottom")
    for field in ("sim_time", "real_time", "real_time_factor", "iterations"):
        add(stats, field, "true")
    add(stats, "topic", f"/world/{world.get('name')}/stats")
    if show_lidar:
        add(gui, "plugin", filename="VisualizeLidar", name="Visualize Lidar")
